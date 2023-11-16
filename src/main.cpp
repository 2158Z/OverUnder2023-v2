#include "main.h"
using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);

ADIDigitalIn cataSwitch('A');
ADIDigitalOut leftPiston('C', false);
ADIDigitalOut rightPiston('B', false);
ADIDigitalOut odomLift('E', false);
Motor cata(2, E_MOTOR_GEAR_RED, true, E_MOTOR_ENCODER_DEGREES);
Motor intakeMotor(16, E_MOTOR_GEAR_RED, true, E_MOTOR_ENCODER_DEGREES);

//left side motor group
Motor leftMotor1(18, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_DEGREES);
Motor leftMotor2(19, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_DEGREES);
Motor leftMotor3(20, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_DEGREES);
MotorGroup leftMotorGroup ({leftMotor1, leftMotor2, leftMotor3});

//right side motor group
Motor rightMotor1(11, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_DEGREES);
Motor rightMotor2(12, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_DEGREES);
Motor rightMotor3(13, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_DEGREES);
MotorGroup rightMotorGroup ({rightMotor1, rightMotor2, rightMotor3});

MotorGroup fullMotorGroup({rightMotor1, rightMotor2, rightMotor3, leftMotor1, leftMotor2, leftMotor3});

IMU inertial(1);
Odom odom;

float wheel_diameter = 3.125;
float wheel_ratio = 0.6;
float gyro_scale;
float drive_in_to_deg_ratio = wheel_ratio/360.0*M_PI*wheel_diameter;
float ForwardTracker_center_distance(ForwardTracker_center_distance);
float ForwardTracker_diameter;
float ForwardTracker_in_to_deg_ratio = M_PI*ForwardTracker_diameter/360.0;
float SidewaysTracker_center_distance;
float SidewaysTracker_diameter;
float SidewaysTracker_in_to_deg_ratio = M_PI*SidewaysTracker_diameter/360.0;

float drive_turn_max_voltage;
float drive_turn_kp;
float drive_turn_ki;
float drive_turn_kd;
float drive_turn_starti;

float drive_turn_settle_error;
float drive_turn_settle_time;
float drive_turn_timeout;

float drive_drive_max_voltage;
float drive_drive_kp;
float drive_drive_ki;
float drive_drive_kd;
float drive_drive_starti;

float drive_drive_settle_error;
float drive_drive_settle_time;
float drive_drive_timeout;

float drive_heading_max_voltage;
float drive_heading_kp;
float drive_heading_ki;
float drive_heading_kd;
float drive_heading_starti;

float drive_swing_max_voltage;
float drive_swing_kp;
float drive_swing_ki;
float drive_swing_kd;
float drive_swing_starti;

float drive_swing_settle_error;
float drive_swing_settle_time;
float drive_swing_timeout;

float drive_desired_heading;

Rotation R_ForwardTracker(1);
Rotation R_SidewaysTracker(2);
ADIDigitalIn E_ForwardTracker(1);
ADIDigitalIn E_SidewaysTracker(2);

// R_ForwardTracker(ForwardTracker_port);
// R_SidewaysTracker(SidewaysTracker_port);
// E_ForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]);
// E_SidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

float get_absolute_heading(){ 
  return( reduce_0_to_360( inertial.get_rotation()*360.0/gyro_scale ) ); 
}

float get_left_position_in(){
  return( leftMotor2.get_position()*drive_in_to_deg_ratio );
}

float get_right_position_in(){
  return( rightMotor2.get_position()*drive_in_to_deg_ratio );
}

float get_X_position(){
  return(odom.X_position);
}

float get_Y_position(){
  return(odom.Y_position);
}

void drive_with_voltage(float leftVoltage, float rightVoltage){
  leftMotorGroup.move_voltage(leftVoltage);
  rightMotorGroup.move_voltage(rightVoltage);
}

void set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  drive_turn_max_voltage = turn_max_voltage;
  drive_turn_kp = turn_kp;
  drive_turn_ki = turn_ki;
  drive_turn_kd = turn_kd;
  drive_turn_starti = turn_starti;
} 

void set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  drive_drive_max_voltage = drive_max_voltage;
  drive_drive_kp = drive_kp;
  drive_drive_ki = drive_ki;
  drive_drive_kd = drive_kd;
  drive_drive_starti = drive_starti;
} 

void set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  drive_heading_max_voltage = heading_max_voltage;
  drive_heading_kp = heading_kp;
  drive_heading_ki = heading_ki;
  drive_heading_kd = heading_kd;
  drive_heading_starti = heading_starti;
}

void set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  drive_swing_max_voltage = swing_max_voltage;
  drive_swing_kp = swing_kp;
  drive_swing_ki = swing_ki;
  drive_swing_kd = swing_kd;
  drive_swing_starti = swing_starti;
} 

void set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  drive_turn_settle_error = turn_settle_error;
  drive_turn_settle_time = turn_settle_time;
  drive_turn_timeout = turn_timeout;
}

void set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_drive_settle_error = drive_settle_error;
  drive_drive_settle_time = drive_settle_time;
  drive_drive_timeout = drive_timeout;
}

void set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  drive_swing_settle_error = swing_settle_error;
  drive_swing_settle_time = swing_settle_time;
  drive_swing_timeout = swing_timeout;
}

void drive_to_point(float X_position, float Y_position, float drive_max_voltage = drive_drive_max_voltage, float heading_max_voltage = drive_heading_max_voltage, float drive_settle_error = drive_drive_settle_error, float drive_settle_time = drive_drive_settle_time, float drive_timeout = drive_drive_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float heading_kp = drive_heading_kp, float heading_ki = drive_heading_ki, float heading_kd = drive_heading_kd, float heading_starti = drive_heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(drivePID.is_settled() == false){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    delay(10);
  }
  drive_desired_heading = get_absolute_heading();
  leftMotorGroup.move_voltage(0);
  rightMotorGroup.move_voltage(0);
}

void turn_to_point(float X_position, float Y_position, float extra_angle_deg = 0, float turn_max_voltage = drive_turn_max_voltage, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_timeout = drive_turn_timeout, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -drive_turn_max_voltage, drive_turn_max_voltage);
    drive_with_voltage(output, -output);
    delay(10);
  }
  drive_desired_heading = get_absolute_heading();
  leftMotorGroup.move_voltage(0);
  rightMotorGroup.move_voltage(0);
}
void right_swing_to_angle(float angle, float swing_max_voltage = drive_swing_max_voltage, float swing_settle_error = drive_swing_settle_error, float swing_settle_time = drive_swing_settle_time, float swing_timeout = drive_swing_timeout, float swing_kp = drive_swing_kp, float swing_ki = drive_swing_ki, float swing_kd = drive_swing_kd, float swing_starti = drive_swing_starti){
  drive_desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -drive_turn_max_voltage, drive_turn_max_voltage);
    rightMotorGroup.move_voltage(-output);
    leftMotorGroup.move_voltage(0);
    delay(10);
  }
  leftMotorGroup.move_voltage(0);
  rightMotorGroup.move_voltage(0);
}

float get_ForwardTracker_position(){
    return(get_right_position_in());
//   if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
//     return(E_ForwardTracker.get_position()*ForwardTracker_in_to_deg_ratio);
//   }else{
//     return(R_ForwardTracker.get_position()*ForwardTracker_in_to_deg_ratio);
//   }
}

float get_SidewaysTracker_position(){
//   if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    return(0);
//   }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
//     return(E_SidewaysTracker.get_position()*SidewaysTracker_in_to_deg_ratio);
//   }else{
//     return(R_SidewaysTracker.get_position()*SidewaysTracker_in_to_deg_ratio);
//   }
}

void set_heading(float orientation_deg){
  inertial.set_heading(orientation_deg*gyro_scale/360.0);
}

void position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
	delay(5);
  }
}

int position_track_task(){
	position_track();
  	return(0);
}

void set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);
  Task odom_task(position_track_task);
}

void left_swing_to_angle(float angle, float swing_max_voltage = drive_swing_max_voltage, float swing_settle_error = drive_swing_settle_error, float swing_settle_time = drive_swing_settle_time, float swing_timeout = drive_swing_timeout, float swing_kp = drive_swing_kp, float swing_ki = drive_swing_ki, float swing_kd = drive_swing_kd, float swing_starti = drive_swing_starti){
  drive_desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -drive_turn_max_voltage, drive_turn_max_voltage);
    leftMotorGroup.move_voltage(output);
    rightMotorGroup.move_voltage(0);
    delay(10);
  }
  leftMotorGroup.move_voltage(0);
  rightMotorGroup.move_voltage(0);
}

void drive_distance(float distance, float heading = drive_desired_heading, float drive_max_voltage = drive_drive_max_voltage, float heading_max_voltage = drive_heading_max_voltage, float drive_settle_error = drive_drive_settle_error, float drive_settle_time = drive_drive_settle_time, float drive_timeout = drive_drive_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float heading_kp = drive_heading_kp, float heading_ki = drive_heading_ki, float heading_kd = drive_heading_kd, float heading_starti = drive_heading_starti){
  drive_desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    printf("f%\n", (drive_error));
    delay(10);
  }
  leftMotorGroup.move_voltage(0);
  rightMotorGroup.move_voltage(0);
}

void turn_to_angle(float angle, float turn_max_voltage = drive_turn_max_voltage, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_timeout = drive_turn_timeout, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti){
  drive_desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    delay(10);
  }
  leftMotorGroup.move_voltage(0);
  rightMotorGroup.move_voltage(0);
}

void initialize() {
  selector::init();

	odom.set_physical_distances(ForwardTracker_center_distance, 0);

	set_drive_constants(10, 1, 0, 0, 0);
	set_heading_constants(6, .4, 0, 1, 0);
	set_turn_constants(12, .4, .03, 3, 15);
	set_swing_constants(12, .3, .001, 2, 15);
	set_drive_exit_conditions(1.5, 300, 5000);
	set_turn_exit_conditions(1, 300, 3000);
	set_swing_exit_conditions(1, 300, 3000);

	leftMotorGroup.set_brake_modes(E_MOTOR_BRAKE_COAST);
  rightMotorGroup.set_brake_modes(E_MOTOR_BRAKE_COAST);
	fullMotorGroup.set_brake_modes(E_MOTOR_BRAKE_COAST);

	inertial.reset();
}

void lowerCata(Motor cata, ADIDigitalIn limit, int volts){
	while (1){
		if (limit.get_value()) {
			break;
		}
		cata.move_voltage(volts);
	}
}

void task_limit(void* param) {
	while(true){
		if (cataSwitch.get_value() == 1){
			cata.tare_position();
		}
		pros::delay(20);
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	switch(selector::auton) {
		case 1:	
      drive_distance(6);
			// drive_distance(24);
			// turn_to_angle(-45);
			// drive_distance(-36);
			// right_swing_to_angle(-90);
			// drive_distance(24);
			// turn_to_angle(0);
			break;
		case 2:
			break;
		case 3:
			break;
		case -1:
			break;
		case -2:
			break;
		case -3:
			break;
		case 0:
			break;
		default:
			break;
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intakeMotor.move_voltage(-11000);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intakeMotor.move_voltage(11000);
		} else {
			intakeMotor.move_voltage(0);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1){
			rightPiston.set_value(true);
		} else {
			rightPiston.set_value(false);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1){
			leftPiston.set_value(true);
		} else {
			leftPiston.set_value(false);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 1){
			cata.move_voltage(12000);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 0){
			cata.move_voltage(0);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
			lowerCata(cata,cataSwitch,10000);
		}

		double leftthrottle = (double) master.get_analog(ANALOG_LEFT_Y)/127;
		double rightthrottle = (double) master.get_analog(ANALOG_RIGHT_X)/127;

		leftMotorGroup.move_voltage((leftthrottle*12000) + (rightthrottle*12000));
		rightMotorGroup.move_voltage((leftthrottle*12000) - (rightthrottle*12000));

		pros::delay(20);
	}
}