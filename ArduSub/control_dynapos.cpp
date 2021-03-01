#include "Sub.h"

#if DYNAPOS_ENABLED == ENABLED

bool Sub::dynapos_init()
{
    
	////////////////////////////////////////////////////////////////
	/* 
	* Check to not initialise if condition not met
	* ALT_HOLD uses !control_check_barometer()
	* POS_HOLD uses !position_ok() && (!visual_odom.enabled() || !visual_odom.healthy()
	* Commented out for SITL testing 
	*/
	/*if (!position_ok() && (!visual_odom.enabled() || !visual_odom.healthy())) {
        return false;
    	}*/
	////////////////////////////////////////////////////////////////
	
	////////////////////////////////////////////////////////////////
	/* 
	* Z controll init
	* Used in POS_HOLD and ALT_HOLD
	* initialize vertical speeds and leash lengths
	* sets the maximum speed up and down returned by position controller
	*/
    	pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    	pos_control.set_max_accel_z(g.pilot_accel_z);
	
	// POS_HOLD by reading accelerometers
    	// initialise position and desired velocity
    	//pos_control.set_alt_target(inertial_nav.get_altitude());   	//1.) or
    	//pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
	
	//vs ALTHOLD which is just the current point?
    	pos_control.set_target_to_stopping_point_z();				//2.)
	////////////////////////////////////////////////////////////////
	
	////////////////////////////////////////////////////////////////
	/* 
	* Attitude
	*/
	// ALT_HOLD for handle_attitude    
    	last_roll = 0;
    	last_pitch = 0;
	// Following 2 not used in POS_HOLD but yaw commands can still be run
	last_yaw = ahrs.yaw_sensor; 				//1.) or
	last_input_ms = AP_HAL::millis();
	// POSHOLD coming from system.cpp and defined in Sub.h
	//last_pilot_heading = ahrs.yaw_sensor;		//2. )
    	////////////////////////////////////////////////////////////////

	// Doesn't appear to be utilised properly in ALT_HOLD or POS_HOLD
    	// holding_depth = true;	

    	////////////////////////////////////////////////////////////////
	/* 
	* XY
	* Need additions/changes to take XY targets
	*/
	// from POSHOLD
	/*pos_control.init_vel_controller_xyz();
    	pos_control.set_desired_velocity_xy(0, 0);
    	pos_control.set_target_to_stopping_point_xy();*/

	//Guided mode controllers for XY control
	//taken from control_guided.cpp init()
	guided_pos_control_start();
	//maybe better to just pull out the initialisations instead of calling it (less dissociated)
    	////////////////////////////////////////////////////////////////
	
	// in POS_HOLD init and run (disarmed) but only in run (disarmed) ALT_HOLD
	attitude_control.set_throttle_out(0.5 ,true, g.throttle_filt);
    	attitude_control.relax_attitude_controllers();
    	pos_control.relax_alt_hold_controllers();
					
    	return true;
}



// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::dynapos_run()
{
    	////////////////////////////////////////////////////////////////
	/*
	* When Unarmed
	*/
    	// disable motors and stabilization
	if (!motors.armed()) {
		motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
		// Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
		attitude_control.set_throttle_out(0.5 ,true, g.throttle_filt);
		attitude_control.relax_attitude_controllers();
		pos_control.relax_alt_hold_controllers();
		last_roll = 0;
		last_pitch = 0;
		last_yaw = ahrs.yaw_sensor;
		// holding_depth = false;
		
		//Only difference between ALT_HOLD and POS_HOLD
		pos_control.set_target_to_stopping_point_xy();
        	return;
    	}
    	////////////////////////////////////////////////////////////////

    	// Vehicle is armed, motors are free to run
    	motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    	////////////////////////////////////////////////////////////////
	/*
	* POS_HOLD HANDLE XY
	*/
	/*
    	pos_control.set_desired_velocity_xy(0,0);

    	if (visual_odom.healthy()) {
        // Allow pilot to reposition the sub
        	if (fabsf(pilot_lateral) > 0.1 || fabsf(pilot_forward) > 0.1) {
            		pos_control.set_target_to_stopping_point_xy();
        	}
        	translate_pos_control_rp(lateral_out, forward_out);
        	// run loiter controller
        	pos_control.update_xy_controller();
    	} 
	else {
        	pos_control.init_vel_controller_xyz();
        	pos_control.set_desired_velocity_xy(0, 0);
       		pos_control.set_target_to_stopping_point_xy();
    	}
    	*/

	//Guided Controller for XY
	//Taken from control_guided.cpp guided_run() whiich is just a switch statement based on the guided mode value 
	//(set as Guided_WP by defualt when using the desired MAVLINK message but skipping that check here and just running the position controller
	//Function written in here and chopped up
	//guided_pos_control_run();
	// run waypoint controller
	// as commands will only be XY , attitude handling is left to the handle_attitude function
	// z control is left as is. 
    	failsafe_terrain_set_status(wp_nav.update_wpnav());

    	float lateral_out, forward_out;
    	translate_wpnav_rp(lateral_out, forward_out);

    	// Send to forward/lateral outputs
    	// motors.set_lateral(lateral_out);
    	// motors.set_forward(forward_out);
	// motor set commands are set combined in below

    	////////////////////////////////////////////////////////////////
    
	////////////////////////////////////////////////////////////////
	/*
	* Function hadles attitude, checks for SET_ATTITUDE_TARGET command
	* Else uses pilots input
	*/
	handle_attitude();
    	////////////////////////////////////////////////////////////////

    	////////////////////////////////////////////////////////////////
	/*
	* Update Z controller
	*/
    	pos_control.update_z_controller();
    	// Read the output of the z controller and rotate it so it always points up
    	Vector3f throttle_vehicle_frame = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(0, 0, motors.get_throttle_in_bidirectional());
    	// Output the Z controller + pilot input to all motors.
    	////////////////////////////////////////////////////////////////

    	////////////////////////////////////////////////////////////////
	/*
	* Send motor commands
	* POS_HOLD also has X Y additions + lateral_out + forward_out (but set to 0)
	*/
    	motors.set_throttle(0.5+throttle_vehicle_frame.z + channel_throttle->norm_input()-0.5);
    	motors.set_forward(-throttle_vehicle_frame.x + forward_out + channel_forward->norm_input());
    	motors.set_lateral(-throttle_vehicle_frame.y + lateral_out + channel_lateral->norm_input());
    	////////////////////////////////////////////////////////////////

   	 ////////////////////////////////////////////////////////////////
	/*
	* Check for and allow pilot to change depth
	*/
    	// We rotate the RC inputs to the earth frame to check if the user is giving an input that would change the depth.
    	Vector3f earth_frame_rc_inputs = ahrs.get_rotation_body_to_ned() * Vector3f(channel_forward->norm_input(), channel_lateral->norm_input(), (2.0f*(-0.5f+channel_throttle->norm_input())));

    	if (fabsf(earth_frame_rc_inputs.z) > 0.05f) { // Throttle input  above 5%
      	// reset z targets to current values
        	pos_control.relax_alt_hold_controllers();
        	pos_control.set_target_to_stopping_point_z();
    	} 
	else { // hold z
        	if (ap.at_surface) {
            		pos_control.set_alt_target(g.surface_depth - 5.0f); // set target to 5cm below surface level
            		//holding_depth = true;
        	} 
		else if (ap.at_bottom) {
            		pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
            		//holding_depth = true;
        	}
    	}
	////////////////////////////////////////////////////////////////
}
#endif  // DYNAPOS_ENABLED == ENABLED
