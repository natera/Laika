/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Overrides the RC channels to take control of the
 *
 * See MESSAGE RC_CHANNELS_OVERRIDE PACKING for more information
 * definition in mavlink_msg_rc_channels_override.h
 *
 * Modifies a mavlink_rc_channels_override_t struct with target RC fake channels
 * chanX_raw value, in microseconds. A value of UINT16_MAX means to ignore this field.
*/
void
override_channel_raw(uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, mavlink_rc_channels_override_t &sp)
{
	sp.chan1_raw = chan1_raw;
	sp.chan2_raw = chan2_raw;
	sp.chan3_raw = chan3_raw;
	sp.chan4_raw = chan4_raw;

	printf("Channels Override 01234 = [ %d , %d , %d , %d ] \n", sp.chan1_raw, sp.chan2_raw, sp.chan3_raw, sp.chan4_raw);
}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// -----------------------------------------------------------------------------
//   Autopilot Interface Class
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//   Con/De structors
// -----------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	//Channels init
	channel_changed = false;
	current_messages.override_channels.chan1_raw = 1500;
	current_messages.override_channels.chan2_raw = 1500;
	current_messages.override_channels.chan3_raw = 1000; //Throttle
	current_messages.override_channels.chan4_raw = 1500;
	current_messages.override_channels.chan5_raw = 1000;
	current_messages.override_channels.chan6_raw = 1000;
	current_messages.override_channels.chan7_raw = 1000;
	current_messages.override_channels.chan8_raw = 1000;


	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Calibrate RC
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
calibrate_RC()
{
	//Set chanels to minimum
	current_messages.override_channels.chan1_raw = 1110;
	current_messages.override_channels.chan2_raw = 1090;
	current_messages.override_channels.chan3_raw = 1111;
	current_messages.override_channels.chan4_raw = 1110;
	current_messages.override_channels.chan5_raw = 1000;
	current_messages.override_channels.chan6_raw = 900;
	current_messages.override_channels.chan7_raw = 900;
	current_messages.override_channels.chan8_raw = 900;
	channel_changed = true;
	//Delay to give time to write
	sleep(5);

	//Set channels to maximum
	current_messages.override_channels.chan1_raw = 1956;
	current_messages.override_channels.chan2_raw = 1950;
	current_messages.override_channels.chan3_raw = 1930;
	current_messages.override_channels.chan4_raw = 1930;
	current_messages.override_channels.chan5_raw = 1600;
	current_messages.override_channels.chan6_raw = 1600;
	current_messages.override_channels.chan7_raw = 1500;
	current_messages.override_channels.chan8_raw = 1500;
	channel_changed = true;
	//Delay to give time to write
	sleep(5);

	//Center channels and throdle to minimum
	current_messages.override_channels.chan1_raw = 1500;
	current_messages.override_channels.chan2_raw = 1500;
	current_messages.override_channels.chan3_raw = 1111; //Throttle //0 Value can recieve signal from the RC
	current_messages.override_channels.chan4_raw = 1500;
	current_messages.override_channels.chan5_raw = 1000;
	current_messages.override_channels.chan6_raw = 1000;
	current_messages.override_channels.chan7_raw = 1000;
	current_messages.override_channels.chan8_raw = 1500;
	channel_changed = true;
	//Delay to give time to write
	sleep(5);
	// while(1)
	// {channel_changed = true;}
}



// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( not received_all and not time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;
			//printf("Message id :%i\n",message.msgid);
			// Handle Message ID
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					char *mode;
					char *status;
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					switch(current_messages.heartbeat.custom_mode)
					{
						case 0:{mode="Stabilize";break;}
						case 1:{mode="Acro";break;}
						case 2:{mode="Altitude Hold";break;}
						case 3:{mode="Auto";break;}
						case 4:{mode="Guided";break;}
						case 5:{mode="Loiter";break;}
						case 6:{mode="RTL";break;}
						case 7:{mode="Circle";break;}
						case 9:{mode="Land";break;}
						case 16:{mode="Position Hold";break;}
					}
					switch(current_messages.heartbeat.system_status)
					{
						case 0:{status="Unkown";break;}
						case 1:{status="Boot";break;}
						case 2:{status="Calibrating";break;}
						case 3:{status="Standby";break;}
						case 4:{status="Active (Motors Engaged)";break;}
						case 5:{status="Critical";break;}
						case 6:{status="Emergency";break;}
					}

					printf("\033[1;1H =======================MAVLINK_MSG_ID_HEARTBEAT=====================\n");
					printf("\033[K Fligh Mode: %i | %s\n", current_messages.heartbeat.custom_mode,mode);
					printf("\033[K MAV Status: %i | %s\n", current_messages.heartbeat.system_status,status);
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_SCALED_IMU2:
				{
					mavlink_msg_scaled_imu2_decode(&message, &(current_messages.scaled_imu2));
					current_messages.time_stamps.scaled_imu2 = get_time_usec();
					this_timestamps.scaled_imu2 = current_messages.time_stamps.scaled_imu2;
					//printf("SCALED IMU2\n");
					//system("clear");
					//printf("X acc (mg): %d\n", current_messages.scaled_imu2.xacc);
					//printf("Y acc (mg): %d\n", current_messages.scaled_imu2.yacc);
					//printf("Z acc (mg): %d\n", current_messages.scaled_imu2.zacc);
					//printf("Time : %d\n", this_timestamps.scaled_imu2);
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//save previous paramters
					float_t pitch,roll,yaw;
					pitch = current_messages.attitude.pitch;
					roll = current_messages.attitude.roll;
					yaw = current_messages.attitude.yaw;
					//Decode message
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					//Check for NaN
					if(current_messages.attitude.pitch != current_messages.attitude.pitch)
						current_messages.attitude.pitch = pitch;
					if(current_messages.attitude.roll != current_messages.attitude.roll)
						current_messages.attitude.roll = roll;
					if(current_messages.attitude.yaw != current_messages.attitude.yaw)
						current_messages.attitude.yaw = yaw;

					printf("\033[5;1H =======================ATTITUDE=====================\n");
					printf("\033[6;1H Pitch (deg): %0.1f\n", ToDeg(current_messages.attitude.pitch));
					printf("\033[7;1H Roll (deg): %0.1f\n", ToDeg(current_messages.attitude.roll));
					printf("\033[8;1H Yaw (deg): %0.1f\n", ToDeg(current_messages.attitude.yaw));
					break;
				}
				case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
				{
					//save previous paramters
					float_t pitch,roll,yaw;
					pitch = current_messages.desired_attitude.nav_pitch;
					roll = current_messages.desired_attitude.nav_roll;
					yaw = current_messages.desired_attitude.nav_bearing;
					//Decode message
					mavlink_msg_nav_controller_output_decode(&message, &(current_messages.desired_attitude));
					//current_messages.time_stamps.attitude = get_time_usec();
					//this_timestamps.attitude = current_messages.time_stamps.attitude;

					printf("\033[6;23H D. Pitch (deg): %0.1f\n", pitch);
					printf("\033[7;23H D. Roll (deg): %0.1f\n", roll);
					printf("\033[8;23H D. Yaw (deg): %0.1f\n", yaw);
					break;
				}

				case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
				{
					mavlink_msg_rc_channels_raw_decode(&message,&(current_messages.current_channels));
					printf("\033[14;1H =======================CURRENT CHANNELS=====================\n");
					printf("\033[K CH1	CH2	CH3	CH4	CH5	CH6	CH7	CH8\n");
					printf("\033[K %i	%i	%i	%i	%i	%i	%i	%i\n",
					current_messages.current_channels.chan1_raw,
					current_messages.current_channels.chan2_raw,
					current_messages.current_channels.chan3_raw,
					current_messages.current_channels.chan4_raw,
					current_messages.current_channels.chan5_raw,
					current_messages.current_channels.chan6_raw,
					current_messages.current_channels.chan7_raw,
					current_messages.current_channels.chan8_raw);
					break;
				}

				case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
				{
					mavlink_msg_servo_output_raw_decode(&message, &(current_messages.current_servo_output));
					printf("\033[10;1H =======================SERVO OUTPUT=====================\n");
					printf("\033[K M3: %i			M1: %i\n M2: %i			M4: %i\n",
						current_messages.current_servo_output.servo3_raw,
						current_messages.current_servo_output.servo1_raw,
						current_messages.current_servo_output.servo2_raw,
						current_messages.current_servo_output.servo4_raw);
					break;
				}


				default:
				{
					//printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		received_all = true;
		// give the write thread time to use the port
		if ( writing_status > false )
			usleep(100); // look for components of batches at 10kHz
	} // end: while not received all
	current_messages.time_stamps.attitude = false;

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_channels()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------
	mavlink_rc_channels_override_t oc = current_messages.override_channels;
	oc.target_system    = 1; //Send command to MAV 001
	oc.target_component = 1; //PX_COMP_ID_ALL
	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;
	mavlink_msg_rc_channels_override_encode(255, 0, &message, &oc);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
	// do the write
	int len = write_message(message);
	// check the write
	if ( not len > 0 )
	{
		fprintf(stderr,"WARNING: could not send OVERRIDE CHANNELS \n");
	}
	else
	{
		channel_changed = false;
	}
	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( not serial_port->status == 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if (result)
		throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while (not current_messages.sysid)
	{
		if (time_to_exit)
			return;
		sleep(1); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}
	// --------------------------------------------------------------------------
	//   PSITION STIMATOR THREAD
	// --------------------------------------------------------------------------

	//printf("START POSITION ESTIMATOR THREAD \n");

	//result = pthread_create( &read_tid, NULL, &start_autopilot_interface_position_estimator_thread, this );
	//if ( result ) throw result;

	// now we're reading messages
	//printf("\n");


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");
	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if (result)
		throw result;

	// wait for it to be started
	while (not writing_status)
	{
		if(time_to_exit)
			return;
		sleep(1); // 10Hz
	}
	// now we're streaming setpoint commands
	printf("\n");
	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   Position Estimator Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_position_estimator_thread(void)
{
		position_estimator_thread();
		return;
}



// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while (not time_to_exit)
	{
		read_messages();
		usleep(100); // Read batches at .1Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while (not time_to_exit)
	{
		usleep(100);   // Stream at 1Hz
		if(channel_changed)
			write_channels();
	}

	// signal end
	writing_status = false;

	return;

}

// ------------------------------------------------------------------------------
//   Position Estimation Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
position_estimator_thread(void)
{
	//Calulate x acceleration component
	float_t xacc = 0;
	float_t yacc = 0;
	float_t zacc = 0;

	float_t vx = 0;
	float_t vy = 0;

	float_t x = 0;
	float_t y = 0;

	float_t pitch = 0;
	float_t roll = 0;
	int i = 0;

	float_t t = 0.1;

	while(1)
	{

		i++;
		pitch = current_messages.attitude.pitch;
		roll = current_messages.attitude.roll;
		if(!(pitch != pitch) && !(roll != roll))
		{
			//system("clear");
			//Calculete global accelerations from NED pitch and roll
			xacc = ((cos(pitch)*current_messages.scaled_imu2.xacc)+(sin(pitch)*current_messages.scaled_imu2.zacc))/100;
			yacc = ((cos(roll)*current_messages.scaled_imu2.yacc)-(sin(roll)*current_messages.scaled_imu2.zacc))/100;
			if(abs(xacc) < 0.5)
				xacc = 0;
			if(abs(yacc) < 0.5)
				yacc = 0;
			//Calculate Velocity
			vx = vx + xacc * t;
			vy = vy + yacc * t;
			if(xacc == 0)
				vx = 0;
			if(yacc == 0)
				vy = 0;
			//Calculate Displacement
			x = x + (vx * t) + (xacc * t * t)/2;
			y = y + (vy * t) + (yacc * t * t)/2;

			//printf("X acc calculada a %0.1f° es: %0.2f | %d\n", pitch*180/pi, xacc, current_messages.scaled_imu2.xacc);
			//printf("Y acc calculada a %0.1f° es: %0.2f | %d\n", roll*180/pi, yacc, current_messages.scaled_imu2.yacc);
			//printf("Vx: %0.0f\n",vx);
			//printf("Vy; %0.0f\n",vy);
			//printf("X: %0.2f\n",x);
			//printf("Y: %0.2f\n",y);

			usleep((int) t*1000000);
		}
	}
	return;
}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_position_estimator_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_position_estimator_thread();

	// done!
	return NULL;
}
