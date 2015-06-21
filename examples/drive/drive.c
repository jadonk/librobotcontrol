 /*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/


/*
	drive.c
	This is a more advanced version of drive_basic which includes the following
	features in addition to normal driving around
	- Steering ackerman based on measured rover geometry.
	- Balancing on 2 wheels
	- automatic arranging of wheels based on how the user orients
	- the rover in balancing mode
	- drive up walls and balance on 2 wheels
*/


/********************************************
* 			Includes & Constants			*
*********************************************/
#include <robotics_cape.h>
#include "drive_config.h"


#define IMU_SAMPLE_RATE_HZ 200	// balancing filter and control loop speed
#define DT 0.005       		// 1/sample_rate
#define SERVO_HZ 50			// drive_stack frequency for driving around

/************************************************************************
* 	arm_state_t
*	ARMED or DISARMED to indicate if the balance controller is running
************************************************************************/
typedef enum arm_state_t{
	DISARMED,
	ARMED
}arm_state_t;

/************************************************************************
* 	orientation_t
*	possible orientations
************************************************************************/
typedef enum orientation_t {
	FLAT,
	LEFT_DOWN,
	RIGHT_DOWN,
	NOSE_DOWN,
	NOSE_UP
}orientation_t;

/************************************************************************
* 	input_mode_t
*	possible modes of user control
*	these are ordered such that DSM2 has highest priority
************************************************************************/
typedef enum input_mode_t {
	NONE,
	MAVLINK,
	BLUETOOTH,
	DSM2
}input_mode_t;

/************************************************************************
* 	drive_mode_t
*	possible modes of driving around
*  note that balancing is dealt with separately
************************************************************************/
typedef enum drive_mode_t {
	LANECHANGE,
	NORMAL_4W,
	CRAB,
	SPIN,
	BALANCE
}drive_mode_t;

/************************************************************************
* 	core_setpoint_t
*	setpoint for the balance controller
*	This is controlled by the drive stack and read by the balance core	
************************************************************************/
typedef struct core_setpoint_t{
	arm_state_t arm_state;	// see arm_state_t declaration
	float theta;			// body lean angle (rad)
	float gamma;			// body turn angle (rad)
	float gamma_dot;		// rate of change of turning
}core_setpoint_t;

/************************************************************************
* 	core_state_t
*	contains workings of the drive stack and information about motor
*	and servo control
*	Should only be written to by the drive_stack thread	
*	also contains information for the balance controller
************************************************************************/
typedef struct core_state_t{
	// outputs to actuators
	float servos[4];
	float motors[4];
	
	// time when core_controller has finished a step
	uint64_t time_us; 
	// inner feedback loop to control theta body angle
	float theta[3];
	float theta_ref[3];
	float current_theta;
	float d_theta;
	float eTheta[3];
	// steering controller for gamma
	float gamma[3];
	float current_gamma;
	float d_gamma;
	float egamma[3];
	// outputs of balance and steering controllers
	float u[3];
	float current_u;
	float duty_split;
	// battery voltage for scaling motor inputs.
	float vBatt; 
	// orientations set by the orientation detection thread
	orientation_t poss_orientation;
	orientation_t orientation;
} core_state_t;



/************************************************************************
* 	user_interface_t
*	represents current command by the user which may be populated from 
*	DSM2, mavlink, bluetooth, or any other communication you like
************************************************************************/
typedef struct user_interface_t{
	// written by the last input watching thread to update
	input_mode_t input_mode;
	drive_mode_t drive_mode;
		
	// All sticks scaled from -1 to 1
	float drive_stick; 	// positive forward
	float turn_stick;	// positive to the right, CW yaw
}user_interface_t;

/************************************************************************
* 	Function declarations in this c file
*	also check out functions in balance_config.h & balance_logging.h		
************************************************************************/
// hardware interrupt routines
int balance_core();

//threads
void* drive_stack(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* dsm2_watcher(void* ptr);
void* orientation_detector(void* ptr);

// regular functions
int on_pause_press();
int print_drive_mode(drive_mode_t mode);
int on_mode_release();
int blink_green();
int blink_red();
int zero_out_controller();
int disarm_controller();
int arm_controller();
int print_orientation(orientation_t orient);
int print_arm_state(arm_state_t arm_state);

/************************************************************************
* 	Global Variables				
************************************************************************/
drive_config_t config;
core_state_t cstate;
user_interface_t user_interface;
core_setpoint_t setpoint;


/***********************************************************************
*	main()
*	start all the threads, and wait still something 
*	triggers a shut down
***********************************************************************/
int main(){
	// initialize cape hardware
	if(initialize_cape()<0){
		blink_red();
		return -1;
	}
	setRED(HIGH);
	setGRN(LOW);
	set_state(UNINITIALIZED);
	
	// set up button handlers first
	// so user can exit by holding pause
	set_pause_pressed_func(&on_pause_press);
	set_mode_unpressed_func(&on_mode_release);
	
	// load data from disk.
	if(load_config(&config)==-1){
		printf("aborting, config file error\n");
		return -1;
	}
	
	// start a thread to slowly sample battery 
	pthread_t  battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}
	
	// start listening for RC control from dsm2 radio
	if(config.enable_dsm2){
		if(initialize_dsm2()<0){
				printf("failed to start DSM2\n");
		}
		else{
			pthread_t  dsm2_thread;
			pthread_create(&dsm2_thread, NULL, dsm2_watcher, (void*) NULL);
		}
	}

	// start the orientation detection thread in the background
	pthread_t orientation_thread;
	pthread_create(&orientation_thread, NULL, orientation_detector, (void*) NULL);
	
	// this thread is in charge of arming and managing the core
	pthread_t  drive_stack_thread;
	pthread_create(&drive_stack_thread, NULL, drive_stack, (void*) NULL);
	
	// initialize the IMU in flat orientation so that
	// flat is with the cape facing up
	signed char imu_orientation[9] = ORIENTATION_FLAT; 
	initialize_imu(IMU_SAMPLE_RATE_HZ, imu_orientation);
	
	//start the interrupt handler
	set_imu_interrupt_func(&balance_core);
	
	// all threads have started, off we go
	set_state(RUNNING);
	setRED(LOW);
	setGRN(HIGH);
	
	// chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	
	cleanup_cape(); // always end with cleanup to shut down cleanly
	return 0;
}

/***********************************************************************
*	drive_stack
*	This is the medium between the user_interface struct and the 
*	physical servos and motors
************************************************************************/
void* drive_stack(void* ptr){
	int i; // general purpose counter for for loops
	float net_drive, net_turn, net_torque_split;

	// exiting condition is checked inside the switch case instead
	while(1){
		switch (get_state()){
		case EXITING:
			return NULL;
			
		case PAUSED:
			disable_motors();
			// not much to do if paused!
			break;
			
		// when running, drive_stack checks if an input mode
		// like mavlink, DSM2, or bluetooth is enabled
		// and moves the servos and motors corresponding to 
		// user input and current controller mode
		case RUNNING:
			enable_motors();
			// now send input to servos and motors based on drive mode and UI
			//  motor & servo orientation: 
			//	left	1 2   right
			// 			4 3	
			
			// if the orientation is flat, drive around on 4 wheels
			if(cstate.poss_orientation==FLAT){
				if(user_interface.input_mode == NONE){
					cstate.servos[0]=config.serv1_center-config.turn_straight;
					cstate.servos[1]=config.serv2_center+config.turn_straight;
					cstate.servos[2]=config.serv3_center-config.turn_straight;
					cstate.servos[3]=config.serv4_center+config.turn_straight;
					for (i=1; i<=4; i++){
						send_servo_pulse_normalized(i,cstate.servos[i-1]);
					}
					disable_motors();
					break;
				}
				switch(user_interface.drive_mode){
				case LANECHANGE:		// lane change maneuver
					net_drive = user_interface.drive_stick*config.motor_max;
					net_turn = user_interface.turn_stick*(0.5-config.turn_straight);
					cstate.motors[0]=net_drive*config.mot1_polarity;
					cstate.motors[1]=net_drive*config.mot2_polarity;
					cstate.motors[2]=net_drive*config.mot3_polarity;
					cstate.motors[3]=net_drive*config.mot4_polarity;
					cstate.servos[0]=config.serv1_center-config.turn_straight+net_turn;
					cstate.servos[1]=config.serv2_center+config.turn_straight+net_turn;
					cstate.servos[2]=config.serv3_center-config.turn_straight+net_turn;
					cstate.servos[3]=config.serv4_center+config.turn_straight+net_turn;
					break;
					
				case NORMAL_4W:		// Normal 4W Steering
					net_drive = user_interface.drive_stick*config.motor_max;
					net_turn = user_interface.turn_stick*config.normal_turn_range;
					net_torque_split = user_interface.turn_stick*config.torque_vec_const*net_drive;
					cstate.motors[0]=(net_drive+net_torque_split)*config.mot1_polarity;
					cstate.motors[1]=(net_drive+net_torque_split)*config.mot2_polarity;
					cstate.motors[2]=(net_drive-net_torque_split)*config.mot3_polarity;
					cstate.motors[3]=(net_drive-net_torque_split)*config.mot4_polarity;
					cstate.servos[0]=config.serv1_center-config.turn_straight+net_turn;
					cstate.servos[1]=config.serv2_center+config.turn_straight+net_turn;
					cstate.servos[2]=config.serv3_center-config.turn_straight-net_turn;
					cstate.servos[3]=config.serv4_center+config.turn_straight-net_turn;
					break;
				
				// crab, turn all wheels sideways and drive
				case CRAB:
					net_drive = user_interface.drive_stick*config.motor_max;
					net_turn = user_interface.turn_stick*config.crab_turn_const\
															*(net_drive+0.5);
					cstate.motors[0]=(net_drive+net_turn)*config.mot1_polarity;
					cstate.motors[1]=-(net_drive+net_turn)*config.mot2_polarity;
					cstate.motors[2]=-(net_drive-net_turn)*config.mot3_polarity;
					cstate.motors[3]=(net_drive-net_turn)*config.mot4_polarity;
					cstate.servos[0]=config.serv1_center+config.turn_crab;
					cstate.servos[1]=config.serv2_center-config.turn_crab;
					cstate.servos[2]=config.serv3_center+config.turn_crab;
					cstate.servos[3]=config.serv4_center-config.turn_crab;
					break;
					
				case SPIN:
					net_drive = user_interface.turn_stick*config.motor_max;
					cstate.motors[0]=net_drive*config.mot1_polarity;
					cstate.motors[1]=-net_drive*config.mot2_polarity;  
					cstate.motors[2]=-net_drive*config.mot3_polarity;
					cstate.motors[3]=net_drive*config.mot4_polarity;
					cstate.servos[0]=config.serv1_center+config.turn_spin;
					cstate.servos[1]=config.serv2_center-config.turn_spin;
					cstate.servos[2]=config.serv3_center+config.turn_spin;
					cstate.servos[3]=config.serv4_center-config.turn_spin;
					break;
					
				default:
					printf("unknown drive_mode\n");
					disable_motors();
					for (i=1; i<=4; i++){
						cstate.motors[i-1]=0;
						cstate.servos[i-1]=0.5;
					}
					break;
				}// end of switch(drive_mode)
				
				// send pulses to servos and drive motors
				for (i=1; i<=4; i++){
					saturate_float(&cstate.servos[i-1],config.turn_min,config.turn_max);
					saturate_float(&cstate.motors[i-1],-config.motor_max,config.motor_max);
					set_motor(i,cstate.motors[i-1]);
					send_servo_pulse_normalized(i,cstate.servos[i-1]);
				}
			}
			// if not flat, orient the motors for balancing
			else{
				switch(cstate.poss_orientation){
				case LEFT_DOWN:
					cstate.servos[0]=config.serv1_center+config.turn_crab;
					cstate.servos[1]=config.serv2_center;
					cstate.servos[2]=config.serv3_center;
					cstate.servos[3]=config.serv4_center-config.turn_crab;
					break;
				case RIGHT_DOWN:
					cstate.servos[0]=config.serv1_center;
					cstate.servos[1]=config.serv2_center-config.turn_crab;
					cstate.servos[2]=config.serv3_center+config.turn_crab;
					cstate.servos[3]=config.serv4_center;
					break;
				case NOSE_DOWN:
					cstate.servos[0]=config.serv1_center-config.turn_straight;
					cstate.servos[1]=config.serv2_center+config.turn_straight;
					cstate.servos[2]=config.serv3_center;
					cstate.servos[3]=config.serv4_center;
					break;
				case NOSE_UP:
					cstate.servos[0]=config.serv1_center;
					cstate.servos[1]=config.serv2_center;
					cstate.servos[2]=config.serv3_center-config.turn_straight;
					cstate.servos[3]=config.serv4_center+config.turn_straight;
					break;
				default: // shouldn't get here
					break;
				}
				// send pulses to servos and drive motors
				for (i=1; i<=4; i++){
					saturate_float(&cstate.servos[i-1],config.turn_min,config.turn_max);
					send_servo_pulse_normalized(i,cstate.servos[i-1]);
				}
			}
	
		default:
			break;
		} // end of switch get_state()
		
		// run about as fast as the core itself 
		usleep(1000000/SERVO_HZ); 
	}
	return NULL;
}

/***********************************************************************
*	print_drive_mode(drive_mode_t mode)
*	prints a readable text name of one of the 4 drive modes
************************************************************************/
int print_drive_mode(drive_mode_t mode){
	switch(mode){
		case LANECHANGE:
			printf("drive_mode: LANECHANGE\n");
			break;
			
		case NORMAL_4W:
			printf("drive_mode: NORMAL_4W\n");
			break;
			
		case CRAB:
			printf("drive_mode: CRAB\n");
			break;
			
		case SPIN:
			printf("drive_mode: SPIN\n");
			break;
			
		default:
			printf("unknown drive_mode\n");
			return -1;
	}
	return 0;
	
}

/************************************************************************
* 	int print_arm_state(arm_state_t arm_state)
*	prints a human readble version of the arm state
************************************************************************/
int print_arm_state(arm_state_t arm_state){
	switch(arm_state){
	case ARMED:
		printf("ARMED");
		break;
	case DISARMED:
		printf("DISARMED");
		break;
	default:
		printf("unknown arm_state");
		return -1;
		break;
	}
	return 0;
}

/************************************************************************
* 	zero_out_controller()
*	clear the controller state and setpoint
*	especially should be called before swapping state to RUNNING
*	keep current theta and vbatt since they may be used by 
*	other threads
************************************************************************/
int zero_out_controller(){
	// wipe setpoint
	setpoint.gamma = 0;
	setpoint.theta = 0;
	cstate.u[0] = 0;
	cstate.u[1] = 0;
	cstate.u[2] = 0;
	cstate.theta_ref[0] = 0;
	cstate.theta_ref[1] = 0;
	cstate.theta_ref[2] = 0;
	cstate.eTheta[0] = 0;
	cstate.eTheta[1] = 0;
	cstate.eTheta[2] = 0;
	cstate.egamma[0] = 0;
	cstate.egamma[1] = 0;
	cstate.egamma[2] = 0;
	
	
	return 0;
}

/************************************************************************
* 	disarm_controller()
*		- disable motors
*		- set the setpoint.core_mode to DISARMED to stop the controller
************************************************************************/
int disarm_controller(){
	disable_motors();
	setpoint.arm_state = DISARMED;
	return 0;
}

/************************************************************************
* 	arm_controller()
*		- zero out the controller
*		- set the setpoint.armed_state to ARMED
*		- enable motors
************************************************************************/
int arm_controller(){
	zero_out_controller();
	setpoint.arm_state = ARMED;
	enable_motors();
	return 0;
}

/***********************************************************************
*	battery_checker()
*	super slow loop checking battery voltage
************************************************************************/
void* battery_checker(void* ptr){
	float new_v;
	while(get_state()!=EXITING){
		new_v = getBattVoltage();
		// check if there is a bad reading
		if (new_v>9.0 || new_v<5.0){
			// printf("problem reading battery\n");
			// use nominal for now
			new_v = config.v_nominal;
		}
		cstate.vBatt = new_v;
		usleep(1000000);
		usleep(1000000);
		usleep(1000000);
	}
	return NULL;
}

/***********************************************************************
*	printf_loop() 
*	prints diagnostics to console
*   this only gets started if executing from terminal
************************************************************************/
void* printf_loop(void* ptr){
	// keep track of last global state variable
	state_t last_state;
	drive_mode_t last_drive_mode = user_interface.drive_mode;
	int print_header_flag = 1;
	print_drive_mode(last_drive_mode);
	
	while(1){
		// check if this is the first time since being paused
		if(get_state()==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING\n");
			print_header_flag=1;
		}
		else if(get_state()==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause button again to start.\n");
		}
		if(user_interface.drive_mode != last_drive_mode){
			printf("\n\n");
			print_drive_mode(user_interface.drive_mode);
			print_header_flag=1;
		}
		last_state = get_state();
		last_drive_mode = user_interface.drive_mode;
		// decide what to print or exit
		switch (get_state()){	
		case RUNNING: { // show all the things
			if(print_header_flag){
				printf("    motors              servos  \n");
				printf(" 1   2   3   4       1   2   3   4\n");
				print_header_flag=0;
			}
			printf("\r");
			printf("%0.1f ", cstate.motors[0]);
			printf("%0.1f ", cstate.motors[1]);
			printf("%0.1f ", cstate.motors[2]);
			printf("%0.1f   ", cstate.motors[3]);
			printf("%0.2f ", cstate.servos[0]);
			printf("%0.2f ", cstate.servos[1]);
			printf("%.2f ", cstate.servos[2]);
			printf("%.2f ", cstate.servos[3]);
			print_orientation(cstate.orientation);
			printf("   "); // clear remaining characters
			fflush(stdout);
			break;
			}
		case PAUSED: {
			break;
			}
		case EXITING:{
			return NULL;
			}
		default: {
			break; // this is only for UNINITIALIZED state
			}
		}
		usleep(200000);
	}
	return NULL;
} 
/***********************************************************************
*	on_pause_press() 
*	Disarm the controller and set system state to paused.
*	If the user holds the pause button for 2 seconds, exit cleanly
************************************************************************/
int on_pause_press(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	switch(get_state()){
	// pause if running
	case EXITING:
		return 0;
	case RUNNING:
		set_state(PAUSED);
		setRED(HIGH);
		setGRN(LOW);
		break;
	case PAUSED:
		set_state(RUNNING);
		setGRN(HIGH);
		setRED(LOW);
		break;
	default:
		break;
	}
	// now wait to see if the user wants to shut down the program
	while(i<samples){
		usleep(us_wait/samples);
		if(get_pause_button_state() == UNPRESSED){
			return 0; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	blink_red();
	set_state(EXITING);
	return 0;
}

/***********************************************************************
*	on_mode_release()
*	placeholder, blinks green led for now
***********************************************************************/
int on_mode_release(){
	blink_green();
	return 0;
}

/***********************************************************************
*	blink_green()
*	nothing exciting, just blink the GRN LED for half a second
*	then return the LED to its original state
***********************************************************************/
int blink_green(){
	// record if the led was on or off so we can return later
	int old_state = getGRN();
	
	const int us_to_blink = 700000; // 0.7 seconds
	const int blink_hz = 10;
	const int delay = 1000000/(2*blink_hz); 
	const int blinks = blink_hz*us_to_blink/1000000;
	int i;
	for(i=0;i<blinks;i++){
		usleep(delay);
		setGRN(!old_state);
		usleep(delay);
		setGRN(old_state);
	}
	return 0;
}

/***********************************************************************
*	blink_red()
*	used to warn user that the program is exiting
***********************************************************************/
int blink_red(){
	const int us_to_blink = 2000000; // 2 seconds
	const int blink_hz = 10;
	const int delay = 1000000/(2*blink_hz); 
	const int blinks = blink_hz*us_to_blink/1000000;
	int i;
	for(i=0;i<blinks;i++){
		usleep(delay);
		setRED(HIGH);
		usleep(delay);
		setRED(LOW);
	}
	return 0;
}


/***********************************************************************
*	dsm2_watcher()
*	listen for RC control for driving around
***********************************************************************/
void* dsm2_watcher(void* ptr){
	const int timeout_frames = 10; // after 10 missed checks, consider broken
	const int check_us = 5000; // dsm2 packets come in at 11ms, check faster
	drive_mode_t temp_drive_mode; // new drive mode selected by user switches
	int missed_frames;
	float turn, drive, switch1, switch2;
	
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
			
			// Read normalized (+-1) inputs from RC radio right stick
			// positive means turn right or go forward
			turn = config.dsm2_turn_polarity * \
					get_dsm2_ch_normalized(config.dsm2_turn_ch);
			drive = config.dsm2_drive_polarity * \
					get_dsm2_ch_normalized(config.dsm2_drive_ch);
			switch1 = config.dsm2_switch1_polarity * \
					get_dsm2_ch_normalized(config.dsm2_switch1_ch);
			switch2 = config.dsm2_switch2_polarity * \
					get_dsm2_ch_normalized(config.dsm2_switch2_ch);
			if(switch1>0 && switch2>0){
				temp_drive_mode = NORMAL_4W;
			}
			else if(switch1>0 && switch2<0){
				temp_drive_mode = LANECHANGE;
			}
			else if(switch1<0 && switch2>0){
				temp_drive_mode = CRAB;
			}
			else if(switch1<0 && switch2<0){
				temp_drive_mode = SPIN;
			}
			else{
				printf("could not interpret DSM2 switches\n");
			}
			
			if(fabs(turn)>1.1 || fabs(drive)>1.1){
				// bad packet, ignore
			}
			else{
				missed_frames = 0;
				// dsm has highest interface priority so take over
				user_interface.input_mode = DSM2;
				user_interface.drive_stick = drive;
				user_interface.turn_stick  = turn;
				user_interface.drive_mode = temp_drive_mode;
			}
			
		}
		// if no new data and currently operating in DSM2 mode, 
		// count a missed frame
		else if(user_interface.input_mode == DSM2){
			missed_frames ++;
			// if enough frames were missed and in DSM2 mode, 
			// this thread relinquishes control 
			if(missed_frames >= timeout_frames){
				user_interface.input_mode = NONE;
			}
		}
		// wait for the next frame
		usleep(check_us); 
	}
	return 0;
}

/************************************************************************
* 	int print_orientation(orientation_t orient)
*	prints a human readble version of the orientation enum
************************************************************************/
int print_orientation(orientation_t orient){
	switch(orient){
	case FLAT:
		printf("   FLAT  ");
		break;
	case LEFT_DOWN:
		printf("LEFT_DOWN");
		break;
	case RIGHT_DOWN:
		printf("RIGHT_DOWN");
		break;
	case NOSE_DOWN:
		printf(" NOSE_DOWN");
		break;
	case NOSE_UP:
		printf(" NOSE_UP  ");
		break;
	default:
		printf("unknown orientation");
		return -1;
		break;
	}
	return 0;
}

/************************************************************************
* 	void* orientation_detector(void* ptr)
*	independent thread that montitors the imu data and determines
*	a possible orientation quickly and a definite orientation more 
* 	slowly with more certainty. 
************************************************************************/
void* orientation_detector(void* ptr){
	// local copies of roll and pitch read from IMU
	float roll, pitch;
	int counter = 0;
	
	// local orientation based on simple imu sample
	orientation_t immediate_orientation;
	
	// counter limits for vertain orientation and possible orientation
	int counter_limit=lrint(config.det_time*config.orientation_rate);
	int counter_poss_limit=lrint(config.det_poss_time*config.orientation_rate);
	
	// run untill the rest of the program closes
	while(get_state()!=EXITING){
		// make local copies of roll and pitch from IMU
		roll = mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE;
		pitch = mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE;
		
		// check for flat
		if(fabs(roll)<(config.orient_tolerance)&& \
				fabs(pitch)<(config.orient_tolerance)){
			if (immediate_orientation != FLAT){
				counter = 0;
				immediate_orientation = FLAT;
			}
			counter ++;
		}
	
		// check for nose down
		else if(pitch<90 && pitch>(90-config.orient_tolerance)){
			if (immediate_orientation != NOSE_DOWN){
				counter = 0;
				immediate_orientation = NOSE_DOWN;
			}
			counter ++;
		}
		
		// check for nose up
		else if(pitch>-90 && pitch<(-90+config.orient_tolerance)){
			if (immediate_orientation != NOSE_UP){
				counter = 0;
				immediate_orientation = NOSE_UP;
			}
			counter ++;
		}
		
		// check for right side down
		else if(roll>-90 && roll<(-90+config.orient_tolerance)){
			if (immediate_orientation != RIGHT_DOWN){
				counter = 0;
				immediate_orientation = RIGHT_DOWN;
			}
			counter ++;
		}

		// check for left side
		else if(roll<90 && roll>(90-config.orient_tolerance)){
			if (immediate_orientation != LEFT_DOWN){
				counter = 0;
				immediate_orientation = LEFT_DOWN;
			}
			counter ++;
		}
		
		
		
		// check for possible counter timeout
		if(counter>=counter_poss_limit){
			cstate.poss_orientation = immediate_orientation;
		}
		
		// check for certain counter timeout
		if(counter>=counter_limit){
			cstate.orientation = immediate_orientation;
			// to prevent the counter from overflowing, stop it here
			counter = counter_limit;
		}
		
		// sleep for roughly enough time to keep the sample rate
		usleep(1000000/config.orientation_rate);
	}
	
	return NULL;
}

/************************************************************************
* 	int balance_core()
************************************************************************/
int balance_core(){
	if(mpu9150_read(&mpu)==0){
		
		
	};
	return 0;
}


