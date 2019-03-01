/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ecl_l1_pos_controller.h
 * Implementation of L1 position control.
 * Authors and acknowledgements in header.
 *
 */

#include <float.h>
#include <systemlib/mavlink_log.h> // (AWE)

#include "ecl_l1_pos_controller.h"

using matrix::Vector2f;
using matrix::wrap_pi;

/* (AWE) */
using matrix::Vector3f;
using matrix::Dcmf;
using matrix::AxisAnglef;
using matrix::wrap_2pi;

void ECL_L1_Pos_Controller::update_roll_setpoint()
{
    float roll_new = atanf(_lateral_accel * 1.0f / CONSTANTS_ONE_G);
    roll_new = math::constrain(roll_new, -_roll_lim_rad, _roll_lim_rad);

    if (_dt > 0.0f && _roll_slew_rate > 0.0f) {
        // slew rate limiting active
        roll_new = math::constrain(roll_new, _roll_setpoint - _roll_slew_rate * _dt, _roll_setpoint + _roll_slew_rate * _dt);
    }

    if (ISFINITE(roll_new)) {
        _roll_setpoint = roll_new;
    }

}

float ECL_L1_Pos_Controller::switch_distance(float wp_radius)
{
    /* following [2], switching on L1 distance */
    return math::min(wp_radius, _L1_distance);
}

void
ECL_L1_Pos_Controller::navigate_waypoints(const Vector2f &vector_A, const Vector2f &vector_B,
                                          const Vector2f &vector_curr_position, const Vector2f &ground_speed_vector)
{
    /* this follows the logic presented in [1] */
    float eta = 0.0f;
    float xtrack_vel = 0.0f;
    float ltrack_vel = 0.0f;

    /* get the direction between the last (visited) and next waypoint */
    _target_bearing = get_bearing_to_next_waypoint((double)vector_curr_position(0), (double)vector_curr_position(1), (double)vector_B(0), (double)vector_B(1));

    /* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
    float ground_speed = math::max(ground_speed_vector.length(), 0.1f);

    /* calculate the L1 length required for the desired period */
    _L1_distance = _L1_ratio * ground_speed;

    /* calculate vector from A to B */
    Vector2f vector_AB = get_local_planar_vector(vector_A, vector_B);

    /*
     * check if waypoints are on top of each other. If yes,
     * skip A and directly continue to B
     */
    if (vector_AB.length() < 1.0e-6f) {
        vector_AB = get_local_planar_vector(vector_curr_position, vector_B);
    }

    vector_AB.normalize();

    /* calculate the vector from waypoint A to the aircraft */
    Vector2f vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position);

    /* calculate crosstrack error (output only) */
    _crosstrack_error = vector_AB % vector_A_to_airplane;

    /*
     * If the current position is in a +-135 degree angle behind waypoint A
     * and further away from A than the L1 distance, then A becomes the L1 point.
     * If the aircraft is already between A and B normal L1 logic is applied.
     */
    float distance_A_to_airplane = vector_A_to_airplane.length();
    float alongTrackDist = vector_A_to_airplane * vector_AB;

    /* estimate airplane position WRT to B */
    Vector2f vector_B_to_P_unit = get_local_planar_vector(vector_B, vector_curr_position).normalized();

    /* calculate angle of airplane position vector relative to line) */

    // XXX this could probably also be based solely on the dot product
    float AB_to_BP_bearing = atan2f(vector_B_to_P_unit % vector_AB, vector_B_to_P_unit * vector_AB);

    /* extension from [2], fly directly to A */
    if (distance_A_to_airplane > _L1_distance && alongTrackDist / math::max(distance_A_to_airplane, 1.0f) < -0.7071f) {

        /* calculate eta to fly to waypoint A */

        /* unit vector from waypoint A to current position */
        Vector2f vector_A_to_airplane_unit = vector_A_to_airplane.normalized();
        /* velocity across / orthogonal to line */
        xtrack_vel = ground_speed_vector % (-vector_A_to_airplane_unit);
        /* velocity along line */
        ltrack_vel = ground_speed_vector * (-vector_A_to_airplane_unit);
        eta = atan2f(xtrack_vel, ltrack_vel);
        /* bearing from current position to L1 point */
        _nav_bearing = atan2f(-vector_A_to_airplane_unit(1), -vector_A_to_airplane_unit(0));

        /*
         * If the AB vector and the vector from B to airplane point in the same
         * direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
         */

    } else if (fabsf(AB_to_BP_bearing) < math::radians(100.0f)) {
        /*
         * Extension, fly back to waypoint.
         *
         * This corner case is possible if the system was following
         * the AB line from waypoint A to waypoint B, then is
         * switched to manual mode (or otherwise misses the waypoint)
         * and behind the waypoint continues to follow the AB line.
         */

        /* calculate eta to fly to waypoint B */

        /* velocity across / orthogonal to line */
        xtrack_vel = ground_speed_vector % (-vector_B_to_P_unit);
        /* velocity along line */
        ltrack_vel = ground_speed_vector * (-vector_B_to_P_unit);
        eta = atan2f(xtrack_vel, ltrack_vel);
        /* bearing from current position to L1 point */
        _nav_bearing = atan2f(-vector_B_to_P_unit(1), -vector_B_to_P_unit(0));

    } else {

        /* calculate eta to fly along the line between A and B */

        /* velocity across / orthogonal to line */
        xtrack_vel = ground_speed_vector % vector_AB;
        /* velocity along line */
        ltrack_vel = ground_speed_vector * vector_AB;
        /* calculate eta2 (angle of velocity vector relative to line) */
        float eta2 = atan2f(xtrack_vel, ltrack_vel);
        /* calculate eta1 (angle to L1 point) */
        float xtrackErr = vector_A_to_airplane % vector_AB;
        float sine_eta1 = xtrackErr / math::max(_L1_distance, 0.1f);
        /* limit output to 45 degrees */
        sine_eta1 = math::constrain(sine_eta1, -0.7071f, 0.7071f); //sin(pi/4) = 0.7071
        float eta1 = asinf(sine_eta1);
        eta = eta1 + eta2;
        /* bearing from current position to L1 point */
        _nav_bearing = atan2f(vector_AB(1), vector_AB(0)) + eta1;

    }

    /* limit angle to +-90 degrees */
    eta = math::constrain(eta, (-M_PI_F) / 2.0f, +M_PI_F / 2.0f);
    _lateral_accel = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);

    /* flying to waypoints, not circling them */
    _circle_mode = false;

    /* the bearing angle, in NED frame */
    _bearing_error = eta;

    update_roll_setpoint();
}

void
ECL_L1_Pos_Controller::navigate_loiter(const Vector2f &vector_A, const Vector2f &vector_curr_position, float radius,
                                       int8_t loiter_direction, const Vector2f &ground_speed_vector)
{
    /* the complete guidance logic in this section was proposed by [2] */

    /* calculate the gains for the PD loop (circle tracking) */
    float omega = (2.0f * M_PI_F / _L1_period);
    float K_crosstrack = omega * omega;
    float K_velocity = 2.0f * _L1_damping * omega;

    /* update bearing to next waypoint */
    _target_bearing = get_bearing_to_next_waypoint((double)vector_curr_position(0), (double)vector_curr_position(1), (double)vector_A(0), (double)vector_A(1));

    /* ground speed, enforce minimum of 0.1 m/s to avoid singularities */
    float ground_speed = math::max(ground_speed_vector.length(), 0.1f);

    /* calculate the L1 length required for the desired period */
    _L1_distance = _L1_ratio * ground_speed;

    /* calculate the vector from waypoint A to current position */
    Vector2f vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position);

    Vector2f vector_A_to_airplane_unit;

    /* prevent NaN when normalizing */
    if (vector_A_to_airplane.length() > FLT_EPSILON) {
        /* store the normalized vector from waypoint A to current position */
        vector_A_to_airplane_unit = vector_A_to_airplane.normalized();

    } else {
        vector_A_to_airplane_unit = vector_A_to_airplane;
    }

    /* calculate eta angle towards the loiter center */

    /* velocity across / orthogonal to line from waypoint to current position */
    float xtrack_vel_center = vector_A_to_airplane_unit % ground_speed_vector;
    /* velocity along line from waypoint to current position */
    float ltrack_vel_center = - (ground_speed_vector * vector_A_to_airplane_unit);
    float eta = atan2f(xtrack_vel_center, ltrack_vel_center);
    /* limit eta to 90 degrees */
    eta = math::constrain(eta, -M_PI_F / 2.0f, +M_PI_F / 2.0f);

    /* calculate the lateral acceleration to capture the center point */
    float lateral_accel_sp_center = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);

    /* for PD control: Calculate radial position and velocity errors */

    /* radial velocity error */
    float xtrack_vel_circle = -ltrack_vel_center;
    /* radial distance from the loiter circle (not center) */
    float xtrack_err_circle = vector_A_to_airplane.length() - radius;

    /* cross track error for feedback */
    _crosstrack_error = xtrack_err_circle;

    /* calculate PD update to circle waypoint */
    float lateral_accel_sp_circle_pd = (xtrack_err_circle * K_crosstrack + xtrack_vel_circle * K_velocity);

    /* calculate velocity on circle / along tangent */
    float tangent_vel = xtrack_vel_center * loiter_direction;

    /* prevent PD output from turning the wrong way */
    if (tangent_vel < 0.0f) {
        lateral_accel_sp_circle_pd = math::max(lateral_accel_sp_circle_pd, 0.0f);
    }

    /* calculate centripetal acceleration setpoint */
    float lateral_accel_sp_circle_centripetal = tangent_vel * tangent_vel / math::max((0.5f * radius),
                                                                                      (radius + xtrack_err_circle));

    /* add PD control on circle and centripetal acceleration for total circle command */
    float lateral_accel_sp_circle = loiter_direction * (lateral_accel_sp_circle_pd + lateral_accel_sp_circle_centripetal);

    /*
     * Switch between circle (loiter) and capture (towards waypoint center) mode when
     * the commands switch over. Only fly towards waypoint if outside the circle.
     */

    // XXX check switch over
    if ((lateral_accel_sp_center < lateral_accel_sp_circle && loiter_direction > 0 && xtrack_err_circle > 0.0f) ||
        (lateral_accel_sp_center > lateral_accel_sp_circle && loiter_direction < 0 && xtrack_err_circle > 0.0f)) {
        _lateral_accel = lateral_accel_sp_center;
        _circle_mode = false;
        /* angle between requested and current velocity vector */
        _bearing_error = eta;
        /* bearing from current position to L1 point */
        _nav_bearing = atan2f(-vector_A_to_airplane_unit(1), -vector_A_to_airplane_unit(0));

    } else {
        _lateral_accel = lateral_accel_sp_circle;
        _circle_mode = true;
        _bearing_error = 0.0f;
        /* bearing from current position to L1 point */
        _nav_bearing = atan2f(-vector_A_to_airplane_unit(1), -vector_A_to_airplane_unit(0));
    }

    update_roll_setpoint();
}

void ECL_L1_Pos_Controller::navigate_heading(float navigation_heading, float current_heading,
                                             const Vector2f &ground_speed_vector)
{
    /* the complete guidance logic in this section was proposed by [2] */

    /*
     * As the commanded heading is the only reference
     * (and no crosstrack correction occurs),
     * target and navigation bearing become the same
     */
    _target_bearing = _nav_bearing = wrap_pi(navigation_heading);

    float eta = wrap_pi(_target_bearing - wrap_pi(current_heading));

    /* consequently the bearing error is exactly eta: */
    _bearing_error = eta;

    /* ground speed is the length of the ground speed vector */
    float ground_speed = ground_speed_vector.length();

    /* adjust L1 distance to keep constant frequency */
    _L1_distance = ground_speed / _heading_omega;
    float omega_vel = ground_speed * _heading_omega;

    /* not circling a waypoint */
    _circle_mode = false;

    /* navigating heading means by definition no crosstrack error */
    _crosstrack_error = 0;

    /* limit eta to 90 degrees */
    eta = math::constrain(eta, (-M_PI_F) / 2.0f, +M_PI_F / 2.0f);
    _lateral_accel = 2.0f * sinf(eta) * omega_vel;

    update_roll_setpoint();
}

void ECL_L1_Pos_Controller::navigate_level_flight(float current_heading)
{
    /* the logic in this section is trivial, but originally proposed by [2] */

    /* reset all heading / error measures resulting in zero roll */
    _target_bearing = current_heading;
    _nav_bearing = current_heading;
    _bearing_error = 0;
    _crosstrack_error = 0;
    _lateral_accel = 0;

    /* not circling a waypoint when flying level */
    _circle_mode = false;

    update_roll_setpoint();
}

Vector2f ECL_L1_Pos_Controller::get_local_planar_vector(const Vector2f &origin, const Vector2f &target) const
{
    /* this is an approximation for small angles, proposed by [2] */
    Vector2f out(math::radians((target(0) - origin(0))),
                 math::radians((target(1) - origin(1))*cosf(math::radians(origin(0)))));

    return out * static_cast<float>(CONSTANTS_RADIUS_OF_EARTH);
}

void ECL_L1_Pos_Controller::set_l1_period(float period)
{
    _L1_period = period;

    /* calculate the ratio introduced in [2] */
    _L1_ratio = 1.0f / M_PI_F * _L1_damping * _L1_period;

    /* calculate normalized frequency for heading tracking */
    _heading_omega = sqrtf(2.0f) * M_PI_F / _L1_period;
}

void ECL_L1_Pos_Controller::set_l1_damping(float damping)
{
    _L1_damping = damping;

    /* calculate the ratio introduced in [2] */
    _L1_ratio = 1.0f / M_PI_F * _L1_damping * _L1_period;

    /* calculate the L1 gain (following [2]) */
    _K_L1 = 4.0f * _L1_damping * _L1_damping;
}

void ECL_L1_Pos_Controller::navigate_awe_circle(const matrix::Vector3f &S2center, const matrix::Vector3f &home_to_S1center,
                                                const int32_t &S2radius_m, const float &theta_r, int8_t loiter_orientation, const matrix::Vector3f &aircraft_curr_pos_global,
                                                const matrix::Vector2f &ground_speed_vector, matrix::Vector3f &desired_loc, float aircraft_yaw)
{
    // CALCULATE DERIVED PARAMETERS FROM ARGUMENT LIST OF THE FUNCTION
    const float cos_thetar = cosf(radians(theta_r));
    const float sin_thetar = sinf(radians(theta_r));
    // distance between the center of the sphere and the center of the circle
    const auto D_m = static_cast<int32_t>((S2radius_m * cos_thetar));  // D_m = [m]
    // radius of the circle
    const auto S1radius_m = static_cast<int32_t>((S2radius_m * sin_thetar)); // S1radius_m = [cm]
    // const Vector3f S2ctoS1cv = ercv * D_m/100.0f;
    // Location of the center of the circle
    Vector3f S1center = S2center;
    location_offset(S1center, home_to_S1center(0) * D_m, home_to_S1center(1) * D_m);   // ToDo: needs testing
    S1center(2) -= home_to_S1center(2) * D_m;
    // trigonometric functions of the polar angle
    const float cos_theta = -home_to_S1center(2);
    const float sin_theta = sqrtf(1.0f - powf(cos_theta,2));
    // calculate desired position on ellipse (= lateral projection of the circle) with major and minor principal axes along unit vectors e1 and e2, respectively
    // position vector of the aircraft parameterized as: posalv(phia) = ra(cos(phia)e1 + cos(theta)sin(phia)e2)
    // trigonometric functions of the azimuth angle
    float cos_psi;
    float sin_psi;
    // if theta == 0, i.e. the circle is horizontal, choose psi = 0
    if (isZero(sin_theta))
    {
        cos_psi = 1;
        sin_psi = 0;
    }
    else
    {
        cos_psi = home_to_S1center(0) / sin_theta;
        sin_psi = home_to_S1center(1) / sin_theta;
    }
    // trigonometric functions of angle at which a circle has to be inclined in order to yield the ellipse as its lateral projection
    // unit vectors e1 and e2 into the direction of the major and minor principal axes, respectively
    //const Vector2f e1(cos_psi,sin_psi);
    //const Vector2f e2(-e1.y,e1.x);
    // minor and major principal axes directions have to be exchanged for the inclined circle
    // this can be accomplished by a 90 degree rotation: e1 -> e2, e2 -> -e1 which preserves the orientation
    const Vector2f e1(-sin_psi,cos_psi); //unit vector pointing along the major principal axis; directed towards east for psi = 0
    const Vector2f e2(-e1(1),e1(0));//unit vector pointing along the minor principal axis; directed towards south for psi = 0
    // minimal height for flying unconstrained outside the sphere
    int32_t heightmin_m = 20;
    // radius (half distance) between the two points of the segment lying above the minimal height
    auto segradius_m = static_cast<int32_t>(sqrt(pow(S1radius_m,2)-pow(heightmin_m,2)));
    // vector is pointing in the direction of motion from start_loc to end_loc on the upper hemicircle (for inclination theta >0) given by orientation
    Vector2f maxv = - e1 * (segradius_m * loiter_orientation);
    Vector3f start_loc = S1center;
    location_offset(start_loc, -maxv(0), -maxv(1));
    start_loc(2) += heightmin_m;
    Vector3f end_loc = S1center;
    location_offset(end_loc, maxv(0), maxv(1));
    end_loc(2) += heightmin_m;

    //DEBUGS:
    //    PX4_INFO("home_to_S1center: %f, %f, %f", static_cast<double>(home_to_S1center(0)), static_cast<double>(home_to_S1center(1)), static_cast<double>(home_to_S1center(2)));
    //	PX4_INFO("S1center: %f, %f, %f", static_cast<double>(S1center(0)), static_cast<double>(S1center(1)), static_cast<double>(S1center(2)));
    //	PX4_INFO("e1: %f, %f", static_cast<double>(e1(0)), static_cast<double>(e1(1)));
    //	PX4_INFO("maxv: %f, %f", static_cast<double>(maxv(0)), static_cast<double>(maxv(1)));
    //	PX4_INFO("start_loc: %f, %f, %f", static_cast<double>(start_loc(0)), static_cast<double>(start_loc(1)), static_cast<double>(start_loc(2)));
    //    PX4_INFO("end_loc: %f, %f, %f", static_cast<double>(end_loc(0)), static_cast<double>(end_loc(1)), static_cast<double>(end_loc(2)));


    // GET CURRENT POSITON AND VELOCITY
    Vector2f S1center_planar(S1center(0),S1center(1));
    Vector2f aircraft_loc_planar(aircraft_curr_pos_global(0),aircraft_curr_pos_global(1));
    // aircraft's position vector from the center of the circle
    Vector2f S1ctoac_planar(get_local_planar_vector(S1center_planar, aircraft_loc_planar));
    Vector3f S1ctoac_3D(S1ctoac_planar(0), S1ctoac_planar(1), aircraft_curr_pos_global(2)-S1center(2));
    // update _target_bearing_cd
    _target_bearing = get_bearing_to_next_waypoint((double)aircraft_curr_pos_global(0), (double)aircraft_curr_pos_global(1), (double)S1center(0), (double)S1center(1));
    // unit tangent vector at point on a circle which is closest to the aircraft; lies in the plane containing the circle
    Vector3f etv = (S1ctoac_3D % home_to_S1center) * loiter_orientation;
    etv = etv.normalized();
    // lateral projection of the unit tangent vector
    Vector2f etlv(etv(0), etv(1));
    etlv = etlv.normalized();
    // outer unit normal (radial) vector at point on circle that is closest to the aircraft
    Vector3f env = (home_to_S1center % etv) * loiter_orientation;
    env = env.normalized(); // renormalize in order to compensate for numerical inaccuracies
    // lateral renormalized projection of the unit normal vector; this is the radial vector of the point of the ellipse (the lateral projection of the circle) but NOT its normal vector;
    Vector2f erlv(env(0), env(1));
    if (erlv.length() > 0.1f) {
        erlv = erlv.normalized();
    }
    else
    {
        if (ground_speed_vector.length() < 0.1f) {
            erlv = Vector2f(cosf(aircraft_yaw), sinf(aircraft_yaw));
        }
        else {
            erlv = ground_speed_vector.normalized();
        }
    }
    // Calculate guidance gains used by PD loop (used during circle tracking)
    const float omega = (M_TWOPI_F / _L1_period);
    const float K_crosstrack = omega * omega;
    const float K_velocity = 2.0f * _L1_damping * omega;
    // ground speed, enforce minimum of 0.1 m/s to avoid singularities
    float ground_speed = math::max(ground_speed_vector.length(), 0.1f);
    // calculate the L1 length required for the desired period
    _L1_distance = _L1_ratio * ground_speed;

    //DEBUG
    // 	PX4_INFO("etlv: %f, %f, %f", static_cast<double>(etlv(0)), static_cast<double>(etlv(1)), static_cast<double>(etlv(2)));
    //	PX4_INFO("erlv: %f, %f, %f", static_cast<double>(erlv(0)), static_cast<double>(erlv(1)), static_cast<double>(erlv(2)));
    //	PX4_INFO("S2ctoav_planar: %f, %f", static_cast<double>(S2ctoav_planar(0)), static_cast<double>(S2ctoav_planar(1)));
    //	PX4_INFO("S1ctoac_planar: %f, %f", static_cast<double>(S1ctoac_planar(0)), static_cast<double>(S1ctoac_planar(1)));


    // CALCULATE DESIRED HEIGHTS - used for circle mode to have the max height closest to home
    //angle on S1 circle between S1center-AC(v1) and direction S1center-home(v2)
    Vector2f v1 = Vector2f(env(0), env(1));
    Vector2f v2 = Vector2f(e2(0), e2(1));
    float skalarprodukt_v1v2 = (v1(0) * v2(0) + v1(1) * v2(1));
    float cos_S1phi = skalarprodukt_v1v2/(v1.length()*v2.length());
    // S1phi is in the range of 0 to 2PI
    float S1phi = acosf(cos_S1phi);
    if (env(1) < 0){
        S1phi = M_TWOPI_F - S1phi;
    }
    // maximum height gain in reference to S1center
    float h_max = sin_theta * (S1radius_m);
    float offset_S1phi = 0.0f;
    if (loiter_orientation > 0) {
        offset_S1phi = M_PI_2_F; //< use only positive values from 0 to 2PI
    }
    float arg = (S1phi - offset_S1phi);
    if (arg < 0){
        arg += M_TWOPI_F;
    }
    // delta_h is maximal when closest to home
    float delta_h_discrete;
    if (arg >= 0 and arg < M_PI_2_F){
        delta_h_discrete = 0.0f;
    } else if (arg >= M_PI_2_F and arg < M_PI_F){
        delta_h_discrete = -h_max;
    } else if (arg >= M_PI_F and arg < 3*M_PI_2_F){
        delta_h_discrete = 0.0f;
    } else {
        delta_h_discrete = h_max;
    }

    // NAVIGATION ON THE LATERAL PROJECTION OF THE INCLINED CICRLE ((DEGENERATE) ELLIPSE)
    // projections of the aircraft's position onto e1 and e2
    const float posal1 = S1ctoac_planar * e1;
    const float posal2 = S1ctoac_planar * e2;
    // distance of the aircraft from the curve;
    float dae;
    // unit tangent vector
    Vector2f etelv;
    // unit outer normal vector at point of the ellipse closest to the aircraft's position relative to center_loc
    Vector2f enelv;
    // curvature at point of the ellipse closest to the aircraft's position relative to center_loc
    float kappa;
    if (!isZero(cos_theta))
    {
        // non-degenerate ellipse
        // distance of the aircraft from the center of the ellipse in meter
        const float ra = sqrtf(powf(posal1,2) + powf((posal2/cos_theta),2));
        const float rho = ra - S1radius_m;
        // trigonometric functions of curve parameter phia at the aircraft's position
        const float cos_phia = posal1/ra;
        const float sin_phia = loiter_orientation * posal2/(ra * cos_theta);
        // first oder correction to curve parameter to approximate parameter at point of the ellipse closest to the aircraft's position
        const float dphi = - rho * powf(sin_theta,2) * sin_phia * cos_phia /(ra * (1 - powf((sin_theta * cos_phia),2)));
        const float cos_dphi = cosf(dphi);
        const float sin_dphi = sinf(dphi);
        // trigonometric functions of phi = phia + dphi, which is the first-order value of the curve parameter of the ellipse at the nearest point of the aircraft
        const float cos_phiapdphi = cos_phia * cos_dphi - sin_phia * sin_dphi;
        const float sin_phiapdphi = cos_phia * sin_dphi + sin_phia * cos_dphi;
        // distance of the aircraft from the ellipse;
        dae = rho * cos_theta /sqrtf(1 - powf((sin_theta * cos_phia),2));
        // position vector of point of the ellipse closest to the aircraft's position relative to center_loc
        const Vector2f telv = Vector2f(-e1 * sin_phiapdphi + e2 * cos_theta * cos_phiapdphi * loiter_orientation);
        const float telvnorm = telv.length();
        // unit tangent vector at point of the ellipse closest to the aircraft's position relative to center_loc
        etelv = telv / telvnorm;
        // unit outer normal vector at point of the ellipse closest to the aircraft's position relative to center_loc
        enelv = Vector2f(etelv(1) * loiter_orientation, -etelv(0) * loiter_orientation);
        // curvature at point of the ellipse closest to the aircraft's position relative to center_loc
        kappa = cos_theta /(ra * powf(telvnorm,3));
        // velocities and accelerations for capturing the center
        // crosstrack lateral velocity: velocity component of the aircraft along tangent vector  <=> velocity perpendicular to the path heading to the target point
        float xtrack_vel_center = ground_speed_vector * etlv * loiter_orientation;
        // along the track lateral velocity: velocity component of the aircraft radial inbound towards the target point
        float ltrack_vel_center = ground_speed_vector * erlv * (-1);
        float eta = atan2f(xtrack_vel_center,ltrack_vel_center);
        eta = math::constrain(eta, -M_PI_2_F, M_PI_2_F); //Limit eta to +- Pi/2
        //Calculate lateral acceleration demand to capture center_WP (use L1 guidance law)
        float lateral_accel_sp_center = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);
        // deviation of the position of the aircraft from the ellipse
        float xtrack_err_circle = dae;
        // keep crosstrack error for reporting
        _crosstrack_error = xtrack_err_circle;
        // lateral velocity component in the direction of the outer normal vector
        float xtrack_vel_circle = ground_speed_vector * enelv;
        // lateral velocity component in the tangential direction of the ellipse
        float ltrack_vel_circle = ground_speed_vector * etelv;
        // calculate lateral acceleration for following the ellipse (the lateral projection of the circle)
        float lateral_accel_sp_circle_centripetal = ltrack_vel_circle * ltrack_vel_circle * kappa;
        // calculate PD control correction to lateral acceleration
        // flight path outside desired circle -> positive correction
        // velocity pointing outwards -> positive correction
        float lateral_accel_sp_circle_pd = xtrack_err_circle * K_crosstrack + xtrack_vel_circle * K_velocity;
        //Calculate tangential velocity
        float tangent_vel = xtrack_vel_center * float(loiter_orientation);
        //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
        if (ltrack_vel_center < 0.0f && tangent_vel < 0.0f) {
            lateral_accel_sp_circle_pd =  max(lateral_accel_sp_circle_pd, 0.0f);
        }
        float tetherErr = 0;
        // sign of lateral acceleration corresponds to sign of lateral_accel_sp_center roll angle
        // roll angle and hence acceleration is negative if orientation is positive
        float lateral_accel_sp_circle = loiter_orientation * (lateral_accel_sp_circle_pd + lateral_accel_sp_circle_centripetal + tetherErr);

        // Perform switchover between 'center' and 'circle' modes at the
        // point where the commands cross over to achieve a seamless transfer
        // Only fly 'center' mode if outside the circle
        if ((lateral_accel_sp_center < lateral_accel_sp_circle && loiter_orientation > 0 && xtrack_err_circle > 0.0f) ||
            (lateral_accel_sp_center > lateral_accel_sp_circle && loiter_orientation < 0 && xtrack_err_circle > 0.0f))
        {
            //			 PX4_INFO("AWE CIRCLE - CENTER MODE");
            _lateral_accel = lateral_accel_sp_center;
            _circle_mode  = false;
            _bearing_error = eta; // angle between demanded and achieved velocity vector, +ve to left of track
            _nav_bearing = 0;//atan2f(-erlv.y , -erlv.x); // bearing (radians) from AC to L1 point
            // desired target: location of point closest to the inclined circle
            desired_loc = S1center;
            location_offset(desired_loc, env(0) * S1radius_m, env(1) * S1radius_m);
            desired_loc(2) = S1center(2) - env(2) * S1radius_m;
        } else {
            //			PX4_INFO("AWE CIRCLE - LOITER MODE");
            _lateral_accel = lateral_accel_sp_circle;
            _circle_mode  = true;
            _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
            _nav_bearing = atan2f(-erlv(1) , -erlv(0)); // bearing (radians)from AC to L1 point
            // desired target: point closest to the inclined circle
            desired_loc = S1center;
            location_offset(desired_loc, env(0) * S1radius_m, env(1) * S1radius_m);
            //			desired_loc(2) = S1center(2) - env(2) * S1radius_m;
            desired_loc(2) = S1center(2) + delta_h_discrete;
        }
    }

    // calc tether length out of distance between aircraft_curr_pos and S2center
    Vector2f S2center_planar(S2center(0),S2center(1));
    Vector2f S2ctoac_planar(get_local_planar_vector(S2center_planar, aircraft_loc_planar));
    Vector3f tether_vector(S2ctoac_planar(0), S2ctoac_planar(1), aircraft_curr_pos_global(2) - S2center(2));
    float tether_length = tether_vector.length();

    struct debug_key_value_s dbg;
    strcpy(dbg.key, "tetherL");
    dbg.value = tether_length;
    orb_advertise(ORB_ID(debug_key_value), &dbg);

    update_roll_setpoint();
}

void ECL_L1_Pos_Controller::navigate_awe_eight(const Vector3f &g_xg, const Vector3f &g_vg, const Vector3f &g_ag,
                                               float &pitch_rate, float &roll_rate, float &thrust_sp, uint8_t &activeWPid, const Vector3f &eulerAngles,
                                               const float flightPlnArr_deg[], const Vector2f &latLonTh_deg, const float &rAtNarrowestTurn,
                                               const float &gain_gammaDiff, const float &gammaOffset_deg, const float &gain_distCorr,
                                               const float &distCorr_tethLen, const float &distCorr_ceta, float &distCorr_omega, orb_advert_t _mavlink_log_pub)
{
    // mavlink_and_console_log_info(&_mavlink_log_pub, "g_xg: %f, %f, %f", (double)g_xg(0), (double)g_xg(1), (double)g_xg(2));
    // mavlink_and_console_log_info(&_mavlink_log_pub, "g_vg: %f, %f, %f", (double)g_vg(0), (double)g_vg(1), (double)g_vg(2));
    // mavlink_and_console_log_info(&_mavlink_log_pub, "g_ag: %f, %f, %f", (double)g_ag(0), (double)g_ag(1), (double)g_ag(2));
    // mavlink_and_console_log_info(&_mavlink_log_pub, "euler: %f, %f, %f", (double)(eulerAngles(0)*M_RAD_TO_DEG_F), (double)(eulerAngles(1)*M_RAD_TO_DEG_F), (double)(eulerAngles(2)*(M_RAD_TO_DEG_F)));

    Vector2f presentLatLon;
    Dcmf Tlg;
    Dcmf Tlk_l;
    float l_gamma;
    float l_chi;

    // Get present position of aircraft relative to sphere
    AWEE_navigation(g_xg, g_vg, presentLatLon, Tlg, Tlk_l, l_gamma, l_chi);
    // mavlink_and_console_log_info(&_mavlink_log_pub, "presentLatLon: %f, %f, %f", (double)presentLatLon(0), (double)presentLatLon(1), (double)presentLatLon(2));
    // mavlink_and_console_log_info(&_mavlink_log_pub, "l_gamma: %f, l_chi: %f,", (double)l_gamma*M_RAD_TO_DEG, (double)l_chi*M_RAD_TO_DEG);

    // Get active waypoint to approach
    Vector3f activeWP;
    AWEE_guidance_WPSource(flightPlnArr_deg, latLonTh_deg, presentLatLon, activeWPid, activeWP);
    // mavlink_and_console_log_info(&_mavlink_log_pub, "activeWP: %f, %f, %f", (double)activeWP(0)*M_RAD_TO_DEG, (double)activeWP(1)*M_RAD_TO_DEG, (double)activeWP(2));

    // Get track towards active waypoint
    float l_chiDiff;
    AWEE_guidance_chiRef(presentLatLon, activeWP, l_chi, l_chiDiff);
    // mavlink_and_console_log_info(&_mavlink_log_pub, "l_chiDiff: %f", (double)l_chiDiff*M_RAD_TO_DEG);

    // Get desired acceleration
    Vector3f g_accRef;
    AWEE_guidance_accRef(l_chiDiff, Tlg, Tlk_l, l_gamma, g_xg, g_vg,
                         rAtNarrowestTurn, gain_gammaDiff, gammaOffset_deg, gain_distCorr, distCorr_tethLen, distCorr_ceta, distCorr_omega, g_accRef);
    // mavlink_and_console_log_info(&_mavlink_log_pub, "g_accRef: %f, %f, %f", (double)g_accRef(0), (double)g_accRef(1), (double)g_accRef(2));


    // Get roll and pitch rates
    AWEE_control(g_accRef, g_vg, g_ag, eulerAngles, roll_rate, pitch_rate, thrust_sp);
    // mavlink_and_console_log_info(&_mavlink_log_pub, "rr: %f, pr: %f, thr: %f", (double)roll_rate*M_RAD_TO_DEG, (double)pitch_rate*M_RAD_TO_DEG, (double)thrust_sp*M_RAD_TO_DEG);

}

/* extrapolate latitude/longitude given distances north and east */
void ECL_L1_Pos_Controller::location_offset(Vector3f &loc, float ofs_north, float ofs_east)
{
    float r_earth_m = 6378000;
    loc(0) += (ofs_north / r_earth_m) * (M_RAD_TO_DEG_F);
    loc(1) += (ofs_east / r_earth_m) * (M_RAD_TO_DEG_F) / cosf(loc(0) * M_DEG_TO_RAD_F);
}



void ECL_L1_Pos_Controller::AWEE_navigation(const Vector3f &g_xg,
                                            const Vector3f &g_vg,
                                            Vector2f &sphLatLon,
                                            matrix::Dcmf &Tlg,
                                            matrix::Dcmf &Tlk_l,
                                            float &l_gamma,
                                            float &l_chi)
{
    /* Local frame on sphere
    local NED frame (letter 'l')
    if tether sphere is a lying "earth ball" with wind blowing from north:
    x: North (latitute on sphere)
    y: East (longitude on sphere)
    z: Down (to sphere center)*/

    // Inputs
    //    Vector3f g_xg(1.0, 1.0, 1.0);
    //    Vector3f g_vg(1.0, 1.0, 1.0);

    // delta: earth latitude, positive lat = north, negative lat = south
    Vector2f g_xg_yz (g_xg(1), g_xg(2));
    float delta = atanf( -g_xg(0) / g_xg_yz.norm() );

    // lambda: earth longitude, positive lon = east, negative lon = west
    float lambda = atanf( -g_xg(1) / -g_xg(2) );

    sphLatLon = Vector2f(delta, lambda);

    // Tlg = T2(-delta) * T1(lambda) * T3(pi) - AxisAngle inverse definition --> Always use negative angles.
    Dcmf Tlg1 (AxisAnglef(Vector3f(0, 1, 0), delta));
    Dcmf Tlg2 (AxisAnglef(Vector3f(1, 0, 0), -lambda));
    Dcmf Tlg3 (AxisAnglef(Vector3f(0, 0, 1), -M_PI_F));
    Tlg = Tlg1 * Tlg2 * Tlg3;

    Vector3f l_vg = Tlg * g_vg;

    // Compute local gamma (flight path) and local chi (flight azimuth) (relative to tangential plane on sphere)
    Vector2f l_vg_xy (l_vg(0), l_vg(1));
    l_gamma = atan2f( -l_vg(2), l_vg_xy.norm() );
    l_chi = atan2f( l_vg(1), l_vg(0));
    l_chi = fmodf(l_chi + M_TWOPI_F, M_TWOPI_F); // Range from 0 to 2pi

    /*Local kinematic frame:
    x: Vk
    (y: right-hand frame)
    z: down in plane through Vk and origin*/

    // Tlk_l = T2(l_gamma) * T3(l_chi) - AxisAngle inverse definition --> Always use negative angles.
    Dcmf Tlk_l1 (AxisAnglef(Vector3f(0, 1, 0), -l_gamma));
    Dcmf Tlk_l2 (AxisAnglef(Vector3f(0, 0, 1), -l_chi));
    Tlk_l = Tlk_l1 * Tlk_l2;

    // Return sphLatLon, Tlg, l_gamma, l_chi, Tlk_l
}

void ECL_L1_Pos_Controller::AWEE_guidance_WPSource(const float flightPlnArr_deg[],
                                                   const Vector2f &latLonTh_deg,
                                                   const Vector2f &presentLatLon,
                                                   uint8_t &activeWPid, Vector3f &activeWP)
{
    matrix::Matrix<float, 4,3> flightPln_deg (flightPlnArr_deg);

    float presentLat = presentLatLon(0);
    float presentLon = presentLatLon(1);

    // 4-Point control switching (AWE Book 2018 Ch. 7)
    float latTh = radians( latLonTh_deg(0) );
    float lonTh = radians( latLonTh_deg(1) );

    switch (activeWPid) {
        case 1:
            if (presentLon > lonTh) activeWPid = 2;
            break;
        case 2:
            if (presentLat < latTh) activeWPid = 3;
            break;
        case 3:
            if (presentLon < -lonTh) activeWPid = 4;
            break;
        case 4:
            if (presentLat < latTh) activeWPid = 1;
            break;
        default:
            activeWPid = 1;
    }

    // Assign activeWP
    activeWP = flightPln_deg.slice<1,3>(static_cast<size_t>(activeWPid - 1), 0).transpose();
    activeWP(0) = radians( activeWP(0) );
    activeWP(1) = radians( activeWP(1) );

    // Return activeWP, activeWPid
}


void ECL_L1_Pos_Controller::AWEE_guidance_chiRef(const Vector2f &presentLatLon,
                                                 const Vector3f &activeWP,
                                                 const float &l_chi,
                                                 float &l_chiDiff)
{
    // Inputs
    Vector3f activeWPLatLonDir = activeWP;

    // Parameters
    float l_chiDiffMax = radians(90.0f);
    //

    // delta (lat), lambda (lon)
    float d1 = presentLatLon(0);
    float l1 = presentLatLon(1);

    float d2 = activeWPLatLonDir(0);
    float l2 = activeWPLatLonDir(1);

    // Bearing for great circle path to WP
    // from https://www.movable-type.co.uk/scripts/latlong.html
    float y = sinf(l2-l1) * cosf(d2);
    float x = cosf(d1) * sinf(d2) - sinf(d1) * cosf(d2) * cosf(l2-l1);
    float l_chiRef = atan2f(y, x);
    l_chiRef = fmodf(l_chiRef + M_TWOPI_F, M_TWOPI_F); // Range from 0 to 2pi

    // Shortest turn to chiRef
    l_chiDiff = l_chiRef - l_chi;

    if ( fabsf(l_chiDiff) > radians(180.0f) ) {

        if (l_chiDiff < 0)
            l_chiDiff  = l_chiDiff + radians(360.0f);
        else
            l_chiDiff = l_chiDiff - radians(360.0f);

    } // l_chiDiff is now the shortest turn difference

    if ( fabsf(l_chiDiff) > l_chiDiffMax ) {
        //  If chiDiff suggests turn of more than 90 deg

        if ( closeEnough(1.0f, activeWPLatLonDir(2)) ) l_chiDiff = l_chiDiffMax; // Make sure it is a right turn
        else if ( closeEnough(-1.0f, activeWPLatLonDir(2)) ) l_chiDiff = -l_chiDiffMax; // Make sure it is a left turn
        else {
            // Turn direction unspecified
            // Limit l_chiDiff to +/- l_chiDiffMax
            if (l_chiDiff < 0)
                l_chiDiff = -l_chiDiffMax;
            else
                l_chiDiff = l_chiDiffMax;
        }

    }

// Return l_chiDiff
}

void ECL_L1_Pos_Controller::AWEE_guidance_accRef(const float &l_chiDiff,
                                                 const matrix::Dcmf &Tlg,
                                                 const matrix::Dcmf &Tlk_l,
                                                 const float &l_gamma,
                                                 const Vector3f &g_xg,
                                                 const Vector3f &g_vg,
                                                 const float &rAtNarrowestTurn,
                                                 const float &gain_gammaDiff,
                                                 const float &gammaOffset_deg,
                                                 const float &gain_distCorr,
                                                 const float &distCorr_tethLen,
                                                 const float &distCorr_ceta,
                                                 const float &distCorr_omega,
                                                 Vector3f &g_accRef)
{
    // Parameters
    float l_chiDiffMax = radians( 90.0f );

    Dcmf Tgl = Tlg.transpose();
    Dcmf Tl_lk = Tlk_l.transpose();

    Vector3f l_vg = Tlg * g_vg;

    // Demanded acceleration in local kinematic frame
    // X
    float lk_accX = 0;

    // Y
    // Acceleration at hardest turn
    float accAtMaxChiDiff = l_vg.norm() * l_vg.norm() / rAtNarrowestTurn;

    // Scale acceleration if turn is less hard
    float lk_accY = accAtMaxChiDiff * l_chiDiff / l_chiDiffMax;

    // Z
    // Feed-forward centripetal acceleration to fly on sphere surface if aircraft is already perfectly on sphere surface
    Vector2f l_vg_xy (l_vg(0), l_vg(1));
    float Z_ff = l_vg_xy.norm() * l_vg_xy.norm() / g_xg.norm();

    // Local flight path angle correction
    float gammaCorr = -l_vg.norm() * (radians( gammaOffset_deg ) - l_gamma);

    // Distance correction
    float distErr = distCorr_tethLen - g_xg.norm();
    distErr = math::constrain(distErr, -10.0f, 10.0f);

    float speedErr = l_vg(2);
    speedErr = math::constrain(speedErr, -5.0f, 5.0f);

    float distCorr = -2.0f * distCorr_omega * distCorr_ceta * speedErr - distCorr_omega*distCorr_omega * distErr;

    float lk_accZ = Z_ff + gain_gammaDiff * gammaCorr + gain_distCorr * distCorr;

    // Assemble acceleration setpoint vector in local kinematic frame
    Vector3f lk_accRef (lk_accX, lk_accY, lk_accZ);

    // Transform to geodaetic frame
    g_accRef = Tgl * Tl_lk * lk_accRef;

    // Return g_accRef
}

void ECL_L1_Pos_Controller::AWEE_control(const Vector3f &g_accRef,
                                         const Vector3f &g_vg,
                                         const Vector3f &g_ag,
                                         const Vector3f &eulerAngles,
                                         float &roll_rate,
                                         float &pitch_rate,
                                         float &thrust_sp)
{

// Inputs
//    Vector3f g_accRef;
//    Vector3f g_acc(0.0,0.0,0.0);
//    Dcmf Tbg;
//    Vector3f eulerAngles; // (Phi, Theta, Psi)
//    Dcmf Tkg; // Or Kinematic angles (Gamma, Chi) Or g_vg

    float Va = g_vg.norm(); // TODO : Get airspeed instead of geodaetic speed

// Parameters
    float FW_PR_MAX_POS_deg = 60.0f;   // TODO EXISTING PARAMETERS
    float FW_PR_MAX_NEG_deg = 60.0f;   // TODO EXISTING PARAMETERS

    float AWEE_R_TC = 0.4f;
    float AWEE_R_LIM_MAX_deg = 60.0f;
    float AWEE_P_LIM_MAX_deg = 60.0f;
    float AWEE_VaMin = 12.0f;  // Exists maybe as a parameter

    // float Va_Ref = 13.0f; // TODO: Implement speed controller
//

    // Calculate flight path angle and azimuth
    Vector2f g_vg_xy (g_vg(0), g_vg(1));
    float gamma = atan2f( -g_vg(2), g_vg_xy.norm() );
    float chi = atan2f( g_vg(1), g_vg(0) );

    // Calculate transformation matrix from geodaetic to kinematic frame
    Dcmf Tkg1 (AxisAnglef(Vector3f(0, 1, 0), -gamma ));
    Dcmf Tkg2 (AxisAnglef(Vector3f(0, 0, 1), -chi ));
    Dcmf Tkg = Tkg1 * Tkg2;

    // Transform to kinematic frame
    Vector3f k_accRef = Tkg * g_accRef;
    Vector3f k_acc = Tkg * g_ag;

    // Acceleration difference and constrain (+/- 2g, Positive z-Dir: 0.8g, shall not be free fall (1g))
    Vector3f k_accDiff = k_accRef - k_acc;

    k_accDiff(0) = math::constrain(k_accDiff(0), -2.0f*9.806f , 2.0f*9.806f);
    k_accDiff(1) = math::constrain(k_accDiff(1), -2.0f*9.806f , 2.0f*9.806f);
    k_accDiff(2) = math::constrain(k_accDiff(2), -2.0f*9.806f , 0.8f*9.806f);

    // Add g compensation
    Vector3f g_gComp (0, 0, -9.81f);
    Vector3f k_gComp = Tkg * g_gComp;
    Vector3f k_accGDiff = k_accDiff + k_gComp;

    // Roll angle and rollRateRef
    float rollAngRef = atan2f(k_accGDiff(1), -k_accGDiff(2));
    rollAngRef = math::constrain(rollAngRef, radians(-AWEE_R_LIM_MAX_deg), radians(AWEE_R_LIM_MAX_deg));

    float rollRateRef = 1/AWEE_R_TC * (rollAngRef - eulerAngles(0));

    // Acceleration in lift axis and pitchRateRef
    Dcmf Tak (matrix::AxisAnglef(Vector3f(1, 0, 0), -eulerAngles(0)));
    Vector3f a_accDiff = Tak * k_accDiff;

    float pitchRateRef = -a_accDiff(2) / max(Va, AWEE_VaMin);
    if ( fabsf(eulerAngles(1)) > radians( AWEE_P_LIM_MAX_deg ) ) pitchRateRef = 0.0f;
    pitchRateRef = math::constrain(pitchRateRef, radians(-FW_PR_MAX_NEG_deg), radians(FW_PR_MAX_POS_deg));

    roll_rate = rollRateRef;
    pitch_rate = pitchRateRef;

    // Thrust setpoint // TODO: Implement speed controller
    thrust_sp = 0.0f;
}


/* test if the floats are so close together that they can be considered equal */
bool ECL_L1_Pos_Controller::closeEnough( float f1, float f2)
{
    return fabsf(f1-f2)<0.001f;
}

bool ECL_L1_Pos_Controller::isZero(float f1)
{
    return closeEnough(f1, 0.00f);
}