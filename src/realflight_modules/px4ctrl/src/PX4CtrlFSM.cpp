#include "PX4CtrlFSM.h"
#include "thrust_estimation_guard.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, Controller &controller_) : param(param_), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

/* 
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

bool PX4CtrlFSM::automaticControlOk(const ros::Time &now)
{
    auto fresh = [&](const ros::Time &stamp, double limit) {
        double age = (now-stamp).toSec(); return !stamp.isZero() && age >= 0. && age <= limit;
    };
    return param.takeoff_land.enable && param.takeoff_land.enable_auto_arm && !param.takeoff_land.no_RC &&
        state_data.current_state.connected && fresh(state_data.rcv_stamp,2.5) && fresh(extended_state_data.rcv_stamp,2.5) &&
        odom_is_received(now) && imu_is_received(now) && rc_is_received(now) &&
        fresh(odom_data.rcv_stamp,param.msg_timeout.odom) && fresh(imu_data.rcv_stamp,param.msg_timeout.imu) &&
        fresh(rc_data.rcv_stamp,param.msg_timeout.rc) &&
        odom_data.msg.header.frame_id == "world" &&
        (now-odom_data.msg.header.stamp).toSec() >= -.02 &&
        (now-odom_data.msg.header.stamp).toSec() <= param.msg_timeout.odom &&
        (now-imu_data.msg.header.stamp).toSec() >= -.02 &&
        (now-imu_data.msg.header.stamp).toSec() <= param.msg_timeout.imu &&
        odom_data.p.allFinite() && odom_data.v.allFinite() && odom_data.q.coeffs().allFinite() &&
        std::abs(odom_data.q.squaredNorm()-1.) < .05 && imu_data.a.allFinite() && imu_data.q.coeffs().allFinite() &&
        rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered();
}

bool PX4CtrlFSM::groundTakeoffInputsOk(const ros::Time &now)
{
    const double bat_age=(now-bat_data.rcv_stamp).toSec();
    const bool ac=automaticControlOk(now);
    const bool armed=state_data.current_state.armed;
    const bool cmd=cmd_is_received(now);
    const bool landed=get_landed();
    const bool ext=extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    const bool ov=odom_data.v.norm() <= .1;
    const bool bat=bat_is_received(now) && bat_age >= 0. &&
        std::isfinite(bat_data.volt) && bat_data.volt > param.low_voltage;
    const bool h=std::isfinite(param.takeoff_land.height) && param.takeoff_land.height > .1;
    ROS_INFO_THROTTLE(1.0, "[px4ctrl] takeoff gate ac=%d armed=%d cmd=%d landed=%d ext=%d ov=%.3f bat=%d volt=%.2f h=%.2f",
        ac, armed, cmd, landed, ext, odom_data.v.norm(), bat, bat_data.volt, param.takeoff_land.height);
    return ac && !armed && !cmd && landed && ext && ov && bat && h;
}

bool PX4CtrlFSM::automaticTakeoffReady(const ros::Time &now)
{
    return param.takeoff_land.require_explicit_takeoff && state == MANUAL_CTRL && groundTakeoffInputsOk(now);
}

void PX4CtrlFSM::process()
{

	ros::Time now_time = ros::Time::now();
    ground_spool_attitude_held = false;
	Controller_Output_t u;
	Desired_State_t des(odom_data);
	bool rotor_low_speed_during_land = state == PREPARING_TAKEOFF;

	// Automatic phases can be interrupted without sending a disarm command in air.
    if (param.takeoff_land.require_explicit_takeoff &&
        (abort_automatic || ((state == PREPARING_TAKEOFF || state == AUTO_TAKEOFF || state == AUTO_LAND) && !automaticControlOk(now_time))))
    {
        abort_automatic = false;
        takeoff_land_data.triggered = false;
        takeoff_land.delay_trigger.first = false;
        if (state == PREPARING_TAKEOFF)
        {
            ROS_WARN("[px4ctrl] Cancel ground takeoff preparation before arming");
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);
        }
        else if ((state == AUTO_TAKEOFF || state == AUTO_LAND) && state_data.current_state.armed &&
            state_data.current_state.mode == "OFFBOARD" && automaticControlOk(now_time))
        { state = AUTO_HOVER; set_hov_with_odom(); }
        else if (state == AUTO_TAKEOFF || state == AUTO_LAND)
        { state = MANUAL_CTRL; toggle_offboard_mode(false); }
    }
    // STEP1: state machine runs
	switch (state)
	{
	case MANUAL_CTRL:
	{
		if (rc_data.enter_hover_mode && (!param.takeoff_land.require_explicit_takeoff || state_data.current_state.armed)) // Try to jump to AUTO_HOVER
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}

			state = AUTO_HOVER;
			controller.resetThrustMapping();
			set_hov_with_odom();
			toggle_offboard_mode(true);

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
		{
			if (param.takeoff_land.require_explicit_takeoff && !automaticTakeoffReady(now_time))
            { ROS_ERROR("[px4ctrl] Reject automatic takeoff: ground preflight is not ready"); break; }
            if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 0.1)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
			if (!get_landed())
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				break;
			}
			if (rc_is_received(now_time)) // Check this only if RC is connected.
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
                    // Reject immediately; do not block the control loop awaiting RC.
					break;
				}
			}

            if (param.takeoff_land.require_explicit_takeoff)
            {
                controller.resetThrustMapping();
                set_start_pose_for_takeoff_land(odom_data);
                takeoff_mode_request_stamp = ros::Time::now();
                takeoff_mode_request_wall = ros::WallTime::now();
                state = PREPARING_TAKEOFF;
                rotor_low_speed_during_land = true;
                if (!toggle_offboard_mode(true)) { state = MANUAL_CTRL; break; }
                ROS_INFO("[px4ctrl] Waiting for fresh OFFBOARD state before arming (maximum 2.5 s)");
                break; // Keep processing sensors, cancellation and setpoints while feedback arrives.
            }
			state = AUTO_TAKEOFF;
			controller.resetThrustMapping();
			set_start_pose_for_takeoff_land(odom_data);
			if (!toggle_offboard_mode(true)) { state = MANUAL_CTRL; break; } // Never arm after rejected mode change
			for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
			{
				ros::Duration(0.01).sleep();
				ros::spinOnce();
			}
			if (param.takeoff_land.require_explicit_takeoff &&
                (abort_automatic || !automaticControlOk(ros::Time::now()) ||
                 state_data.current_state.armed || state_data.current_state.mode != "OFFBOARD" ||
                 extended_state_data.current_extended_state.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND))
            { state = MANUAL_CTRL; takeoff_land_data.triggered = false; toggle_offboard_mode(false); break; }
			if (param.takeoff_land.enable_auto_arm)
			{
				if (!toggle_arm_disarm(true)) { state = MANUAL_CTRL; toggle_offboard_mode(false); break; }
			}
			takeoff_land.toggle_takeoff_land_time = now_time;

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}

		if (rc_data.toggle_reboot && !param.takeoff_land.require_explicit_takeoff) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		{
			if (state_data.current_state.armed)
			{
				ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}

		break;
	}

    case PREPARING_TAKEOFF:
    {
        rotor_low_speed_during_land = true;
        if (!groundTakeoffInputsOk(now_time) ||
            (odom_data.p-takeoff_land.start_pose.head<3>()).norm() > .10)
        {
            ROS_ERROR("[px4ctrl] Reject takeoff before arming: ground/RC/sensor/battery/position check changed");
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);
            break;
        }
        if ((ros::WallTime::now()-takeoff_mode_request_wall).toSec() > 2.5)
        {
            ROS_ERROR("[px4ctrl] Reject takeoff before arming: fresh OFFBOARD feedback timeout; observed mode=%s",
                      state_data.current_state.mode.c_str());
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);
            break;
        }
        if (state_data.rcv_stamp < takeoff_mode_request_stamp || state_data.current_state.mode != "OFFBOARD")
            break;
        // The mode service only acknowledges sending a request. Arm once, only after
        // fresh FCU feedback and a new complete ground-preflight check in this cycle.
        if (!toggle_arm_disarm(true))
        {
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);
            break;
        }
        state = AUTO_TAKEOFF;
        takeoff_land.toggle_takeoff_land_time = ros::Time::now();
        ROS_INFO("[px4ctrl] Fresh OFFBOARD confirmed; arm accepted; PREPARING_TAKEOFF --> AUTO_TAKEOFF");
        break;
    }

	case AUTO_HOVER:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		else if (rc_data.is_command_mode && cmd_is_received(now_time))
		{
			if (state_data.current_state.mode == "OFFBOARD")
			{
				state = CMD_CTRL;
				des = get_cmd_des();
				ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{

			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);

			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		else
		{
			set_hov_with_rc();
			des = get_hover_des();
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
			}

			// cout << "des.p=" << des.p.transpose() << endl;
		}

		break;
	}

	case CMD_CTRL:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else
		{
			des = get_cmd_des();
		}

		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
					  param.msg_timeout.cmd);
		}

		break;
	}

	case AUTO_TAKEOFF:
	{
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
		{
			des = get_rotor_speed_up_des(now_time);
            ground_spool_attitude_held = param.takeoff_land.require_explicit_takeoff &&
                param.takeoff_land.hold_attitude_during_spool;
		}
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}

	case AUTO_LAND:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_WARN("[px4ctrl] AUTO_LAND -> AUTO_HOVER because the RC command-mode "
					 "switch is OFF; RC hover/manual landing control is active.");
		}
		else if (!get_landed())
		{
			des = get_takeoff_land_des(-param.takeoff_land.speed);
		}
		else
		{
			rotor_low_speed_during_land = true;

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
				print_once_flag = false;
			}

			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
			{
				static double last_trial_time = 0; // Avoid too frequent calls
				if (now_time.toSec() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false)) // disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false); // toggle off offboard after disarm
						ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.toSec();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// STEP2: solve and update new control commands
	if (rotor_low_speed_during_land) // used at the start of auto takeoff
	{
		motors_idling(imu_data, u);
	}
	else
	{
		switch (param.pose_solver)
		{
		case 0:
			debug_msg = controller.update_alg0(des, odom_data, imu_data, u, bat_data.volt);
			debug_msg.header.stamp = now_time;
			debug_pub.publish(debug_msg);
			break;
		case 1:
			debug_msg = controller.update_alg1(des, odom_data, imu_data, u, bat_data.volt);
			debug_msg.header.stamp = now_time;
			debug_pub.publish(debug_msg);
			break;

		case 2:
			controller.update_alg2(des, odom_data, imu_data, u, bat_data.volt);
			break;

		default:
			ROS_ERROR("Illegal pose_slover selection!");
			return;
		}
	}

	// STEP3: estimate thrust model
	if (px4ctrl_guard::allowThrustEstimation(
            state == AUTO_HOVER || state == CMD_CTRL,
            state_data.current_state.connected,
            state_data.current_state.armed,
            state_data.current_state.mode == "OFFBOARD",
            extended_state_data.current_extended_state.landed_state ==
                mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR,
            (now_time - state_data.rcv_stamp).toSec(),
            (now_time - extended_state_data.rcv_stamp).toSec()))
	{
		controller.estimateThrustModel(imu_data.a, bat_data.volt, odom_data.v, param);
	}

	// STEP4: publish control commands to mavros
    if (ground_spool_attitude_held)
    {
        // Candidate policy: while the existing 3 s spool timer is active, avoid
        // requesting tilt corrections that a ground-constrained vehicle cannot
        // follow. Preserve the existing collective-thrust calculation exactly.
        // The debug message above still describes the computed PID proposal;
        // AttitudeTarget is the applied output, with the hold status reported.
        u.q = imu_data.q;
        u.bodyrates.setZero();
    }
	publish_attitude_ctrl(u, now_time);
	
	// STEP5: Detect if the drone has landed
	land_detector(state, des, odom_data);
	// cout << takeoff_land.landed << " ";
	// fflush(stdout);

	// STEP6: Clear flags beyound their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
	u.q = imu.q;
	u.bodyrates = Eigen::Vector3d::Zero();
	u.thrust = 0.04;
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if ((last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF)) ||
        (last_state == State_t::PREPARING_TAKEOFF && state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false; // Always holds
	}
	last_state = state;

	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return; // No need of other decisions
	}

	// land_detector parameters
	constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
	constexpr double VELOCITY_THR_C = 0.15;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
	constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

	static ros::Time time_C12_reached; // time_Constraints12_reached
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) //Constraint 3 reached
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;

	return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
	if (des_a_z > 0.1)
	{
		ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
	// takeoff_land.last_set_cmd_time = now;

	// takeoff_land.start_pose(2) += speed * delta_t;

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;

	// if (param.print_dbg)
	// {
	// 	static unsigned int count = 0;
	// 	if (count++ % 100 == 0)
	// 	{
	// 		cout << "hover_pose=" << hover_pose.transpose() << endl;
	// 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
	// 	}
	// }
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;

	if (on_off)
	{
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
			state_data.state_before_offboard.mode = "MANUAL";

		offb_set_mode.request.custom_mode = "OFFBOARD";
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}

	return true;

	// if (param.print_dbg)
	// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4! Kill-switch activated?");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}

void PX4CtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}
