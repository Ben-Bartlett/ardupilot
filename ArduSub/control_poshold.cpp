// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"

#if POSHOLD_ENABLED == ENABLED

// poshold_init - initialise PosHold controller
bool Sub::poshold_init()
{
    // fail to initialise PosHold mode if no GPS lock
    
    if (!position_ok() && (!visual_odom.enabled() || !visual_odom.healthy())) {
        return false;
    }
    pos_control.init_vel_controller_xyz();
    pos_control.set_desired_velocity_xy(0, 0);
    pos_control.set_target_to_stopping_point_xy();

    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    attitude_control.set_throttle_out(0.5 ,true, g.throttle_filt);
    attitude_control.relax_attitude_controllers();
    pos_control.relax_alt_hold_controllers();

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Sub::poshold_run()
{
    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0.5 ,true, g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.set_target_to_stopping_point_xy();
        pos_control.relax_alt_hold_controllers();
        last_roll = 0;
        last_pitch = 0;
        last_yaw = ahrs.yaw_sensor;
        holding_depth = false;
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    ///////////////////////
    // update xy outputs //
    float pilot_lateral = channel_lateral->norm_input();
    float pilot_forward = channel_forward->norm_input();

    float lateral_out = 0;
    float forward_out = 0;

    pos_control.set_desired_velocity_xy(0,0);

    // Allow pilot to reposition the sub
    if (fabsf(pilot_lateral) > 0.1 || fabsf(pilot_forward) > 0.1) {
        pos_control.set_target_to_stopping_point_xy();
    }
    translate_pos_control_rp(lateral_out, forward_out);
    // run loiter controller
    pos_control.update_xy_controller();

    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    handle_attitude();

    pos_control.update_z_controller();
    // Read the output of the z controller and rotate it so it always points up
    Vector3f throttle_vehicle_frame = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(0, 0, motors.get_throttle_in_bidirectional());
    // Output the Z controller + pilot input to all motors.

    //TODO: scale throttle with the ammount of thrusters in the given direction
    motors.set_throttle(0.5+throttle_vehicle_frame.z + channel_throttle->norm_input()-0.5);
    motors.set_forward(-throttle_vehicle_frame.x + forward_out + pilot_forward);
    motors.set_lateral(-throttle_vehicle_frame.y + lateral_out + pilot_lateral);

    // We rotate the RC inputs to the earth frame to check if the user is giving an input that would change the depth.
    Vector3f earth_frame_rc_inputs = ahrs.get_rotation_body_to_ned() * Vector3f(channel_forward->norm_input(), channel_lateral->norm_input(), (2.0f*(-0.5f+channel_throttle->norm_input())));

    if (fabsf(earth_frame_rc_inputs.z) > 0.05f) { // Throttle input  above 5%
       // reset z targets to current values
        holding_depth = false;
        pos_control.relax_alt_hold_controllers();
    } else { // hold z
        if (ap.at_surface) {
            pos_control.set_alt_target(g.surface_depth - 5.0f); // set target to 5cm below surface level
            holding_depth = true;
        } else if (ap.at_bottom) {
            pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
            holding_depth = true;
        } else if (!holding_depth) {
            pos_control.set_target_to_stopping_point_z();
            holding_depth = true;
        }
    }
}
#endif  // POSHOLD_ENABLED == ENABLED
