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
 * @file ecl_l1_pos_control.h
 * Implementation of L1 position control.
 *
 *
 * Acknowledgements and References:
 *
 *    This implementation has been built for PX4 based on the original
 *    publication from [1] and does include a lot of the ideas (not code)
 *    from [2].
 *
 *
 *    [1] S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *    Proceedings of the AIAA Guidance, Navigation and Control
 *    Conference, Aug 2004. AIAA-2004-4900.
 *
 *    [2] Paul Riseborough, Brandon Jones and Andrew Tridgell, L1 control for APM. Aug 2013.
 *     - Explicit control over frequency and damping
 *     - Explicit control over track capture angle
 *     - Ability to use loiter radius smaller than L1 length
 *     - Modified to use PD control for circle tracking to enable loiter radius less than L1 length
 *     - Modified to enable period and damping of guidance loop to be set explicitly
 *     - Modified to provide explicit control over capture angle
 *
 */

#ifndef ECL_L1_POS_CONTROLLER_H
#define ECL_L1_POS_CONTROLLER_H

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <geo/geo.h>
#include <ecl.h>
#include <uORB/uORB.h> // (AWE)
#include <uORB/topics/debug_key_value.h> // (AWE)

/* (AWE) */
using math::max;
using math::min;
using math::radians;
using matrix::Vector2f;
using matrix::Vector3f;

/**
 * L1 Nonlinear Guidance Logic
 */
class ECL_L1_Pos_Controller
{
public:
	/**
	 * The current target bearing
	 *
	 * @return bearing angle (-pi..pi, in NED frame)
	 */
	float nav_bearing() { return matrix::wrap_pi(_nav_bearing); }

	/**
	 * Get lateral acceleration demand.
	 *
	 * @return Lateral acceleration in m/s^2
	 */
	float nav_lateral_acceleration_demand() { return _lateral_accel; }

	/**
	 * Heading error.
	 *
	 * The heading error is either compared to the current track
	 * or to the tangent of the current loiter radius.
	 */
	float bearing_error() { return _bearing_error; }

	/**
	 * Bearing from aircraft to current target.
	 *
	 * @return bearing angle (-pi..pi, in NED frame)
	 */
	float target_bearing() { return _target_bearing; }

	/**
	 * Get roll angle setpoint for fixed wing.
	 *
	 * @return Roll angle (in NED frame)
	 */
	float get_roll_setpoint(){ return _roll_setpoint; }

	/**
	 * Get the current crosstrack error.
	 *
	 * @return Crosstrack error in meters.
	 */
	float crosstrack_error() { return _crosstrack_error; }

	/**
	 * Returns true if the loiter waypoint has been reached
	 */
	bool reached_loiter_target() { return _circle_mode; }

	/**
	 * Returns true if following a circle (loiter)
	 */
	bool circle_mode() { return _circle_mode; }

	/**
	 * Get the switch distance
	 *
	 * This is the distance at which the system will
	 * switch to the next waypoint. This depends on the
	 * period and damping
	 *
	 * @param waypoint_switch_radius The switching radius the waypoint has set.
	 */
	float switch_distance(float waypoint_switch_radius);

	/**
	 * Navigate between two waypoints
	 *
	 * Calling this function with two waypoints results in the
	 * control outputs to fly to the line segment defined by
	 * the points and once captured following the line segment.
	 * This follows the logic in [1].
	 *
	 * @return sets _lateral_accel setpoint
	 */
	void navigate_waypoints(const matrix::Vector2f &vector_A, const matrix::Vector2f &vector_B,
							const matrix::Vector2f &vector_curr_position, const matrix::Vector2f &ground_speed);

	/**
	 * Navigate on an orbit around a loiter waypoint.
	 *
	 * This allow orbits smaller than the L1 length,
	 * this modification was introduced in [2].
	 *
	 * @return sets _lateral_accel setpoint
	 */
	void navigate_loiter(const matrix::Vector2f &vector_A, const matrix::Vector2f &vector_curr_position, float radius,
						 int8_t loiter_direction, const matrix::Vector2f &ground_speed_vector);

	/**
	* AWE Mode to fly the 8 Pattern in the hemisphere.
	*
	* Own implementation - SWE University of Stuttgart Feb 2019
	*/
	void navigate_awe_eight(const matrix::Vector3f &g_xg, const matrix::Vector3f &g_vg, const matrix::Vector3f &g_ag,
							float &pitch_rate, float &roll_rate, float &thrust_sp, uint8_t &activeWPid, const Vector3f &eulerAngles,
							const float flightPlnArr_deg[], const Vector2f &latLonTh_deg, const float &rAtNarrowestTurn,
							const float &gain_gammaDiff, const float &gammaOffset_deg, const float &gain_distCorr,
							const float &distCorr_tethLen, const float &distCorr_ceta, float &distCorr_omega);

	/**
	 * AWE Mode to fly an inclined circle in the hemisphere. Inspired by the AWEsome project of the University of Bonn.
	 * https://www.lhc-ilc.physik.uni-bonn.de/research-groups/experimental-physics/prof.-k.-desch/research/airborne-wind-energy?set_language=en/
	 *
	 * If aircraft is outside of the S1 circle it will be in the 'center mode' and will fly towards the center of S1. If
	 * it reaches the circle we will switch to 'circle mode' and fly the inclined circle S1.
	 * For now the parameters which define the different circles are hardcoded into 'FixedwingPositionControll.cpp'.
	 *
	 * @param S2center: center of the home base
	 * @param home_to_S1center: vector pointing from home to the center of the inclined circle (S1)
	 * @param S2radius_m: radius of S2, equals optimal tether length
	 * @param theta_r: cone angle of S1 to S2 connection, defines radius of S1
	 * @param loiter_orientation: direction of loiter
	 * @param aircraft_curr_pos_global: current lat, lon and alt of aircraft
	 * @param ground_speed_vector: current vel_north and vel_east of aircraft
	 * @param desired_loc: point on S1 circle closest to the aircraft
	 * @param aircraft_yaw: current yaw angle of aircraft
	 */
	void navigate_awe_circle(const matrix::Vector3f &S2center, const matrix::Vector3f &home_to_S1center, const int32_t &S2radius_m,
							 const float &theta_r, int8_t loiter_orientation, const matrix::Vector3f &aircraft_curr_pos_global,
							 const matrix::Vector2f &ground_speed_vector, matrix::Vector3f &desired_loc, float aircraft_yaw);

	/**
	 * Navigate on a fixed bearing.
	 *
	 * This only holds a certain direction and does not perform cross
	 * track correction. Helpful for semi-autonomous modes. Introduced
	 * by [2].
	 *
	 * @return sets _lateral_accel setpoint
	 */
	void navigate_heading(float navigation_heading, float current_heading, const matrix::Vector2f &ground_speed);

	/**
	 * Keep the wings level.
	 *
	 * This is typically needed for maximum-lift-demand situations,
	 * such as takeoff or near stall. Introduced in [2].
	 */
	void navigate_level_flight(float current_heading);

	/**
	 * Set the L1 period.
	 */
	void set_l1_period(float period);

	/**
	 * Set the L1 damping factor.
	 *
	 * The original publication recommends a default of sqrt(2) / 2 = 0.707
	 */
	void set_l1_damping(float damping);

	/**
	 * Set the maximum roll angle output in radians
	 */
	void set_l1_roll_limit(float roll_lim_rad) { _roll_lim_rad = roll_lim_rad; }

	/**
	 * Set roll angle slew rate. Set to zero to deactivate.
	 */
	void set_roll_slew_rate(float roll_slew_rate) { _roll_slew_rate = roll_slew_rate; }

	/**
	 * Set control loop dt. The value will be used to apply roll angle setpoint slew rate limiting.
	 */
	void set_dt(float dt) { _dt = dt;}

private:

	float _lateral_accel{0.0f};		///< Lateral acceleration setpoint in m/s^2
	float _L1_distance{20.0f};		///< L1 lead distance, defined by period and damping
	bool _circle_mode{false};		///< flag for loiter mode
	float _nav_bearing{0.0f};		///< bearing to L1 reference point
	float _bearing_error{0.0f};		///< bearing error
	float _crosstrack_error{0.0f};	///< crosstrack error in meters
	float _target_bearing{0.0f};		///< the heading setpoint

	float _L1_period{25.0f};		///< L1 tracking period in seconds
	float _L1_damping{0.75f};		///< L1 damping ratio
	float _L1_ratio{5.0f};		///< L1 ratio for navigation
	float _K_L1{2.0f};			///< L1 control gain for _L1_damping
	float _heading_omega{1.0f};		///< Normalized frequency

	float _roll_lim_rad{math::radians(30.0f)};  ///<maximum roll angle in radians
	float _roll_setpoint{0.0f};	///< current roll angle setpoint in radians
	float _roll_slew_rate{0.0f};	///< roll angle setpoint slew rate limit in rad/s
	float _dt{0};				///< control loop time in seconds

	/**
	 * Convert a 2D vector from WGS84 to planar coordinates.
	 *
	 * This converts from latitude and longitude to planar
	 * coordinates with (0,0) being at the position of ref and
	 * returns a vector in meters towards wp.
	 *
	 * @param ref The reference position in WGS84 coordinates
	 * @param wp The point to convert to into the local coordinates, in WGS84 coordinates
	 * @return The vector in meters pointing from the reference position to the coordinates
	 */
	matrix::Vector2f get_local_planar_vector(const matrix::Vector2f &origin, const matrix::Vector2f &target) const;

	/**
	 * Update roll angle setpoint. This will also apply slew rate limits if set.
	 *
	 */
	void update_roll_setpoint();

	/* (AWE) helper functions */
	void location_offset(Vector3f& loc, float ofs_north, float ofs_east);

	void AWEE_navigation(const Vector3f &g_xg,
						 const Vector3f &g_vg,
						 Vector2f &sphLatLon,
						 matrix::Dcmf &Tlg,
						 matrix::Dcmf &Tlk_l,
						 float &l_gamma,
						 float &l_chi);

	void AWEE_guidance_WPSource(const float flightPlnArr_deg[],
								const Vector2f &latLonTh_deg,
								const Vector2f &presentLatLon,
								uint8_t &activeWPid,
								Vector3f &activeWP);

	void AWEE_guidance_chiRef(const Vector2f &presentLatLon,
							  const Vector3f &activeWP,
							  const float &l_chi,
							  float &l_chiDiff);

	void AWEE_guidance_accRef(const float &l_chiDiff,
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
							  Vector3f &g_accRef);

	void AWEE_control(const Vector3f &g_accRef,
					  const Vector3f &g_vg,
					  const Vector3f &g_ag,
					  const Vector3f &eulerAngles,
					  float &roll_rate,
					  float &pitch_rate,
					  float &thrust_sp);

	bool closeEnough( float f1, float f2);
	bool isZero(float f1);

};


#endif /* ECL_L1_POS_CONTROLLER_H */
