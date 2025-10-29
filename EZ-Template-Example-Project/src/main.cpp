#include "main.h"

// =========================
//  BUTTON & DEVICE DEFINES
// =========================
#define PushUpMove DIGITAL_R1
#define PushMiddleMove DIGITAL_R2
#define PushDownMove DIGITAL_L1
#define TakeInMove DIGITAL_L2
#define StopIntakeMove DIGITAL_X
#define ArmLoaderAirCompressor DIGITAL_A
#define MatchLoaderAirCompressor DIGITAL_B
#define IncreaseDriveSpeed DIGITAL_UP
#define DecreaseDriveSpeed DIGITAL_DOWN
#define IncreaseTurnSpeed DIGITAL_RIGHT
#define DecreaseTurnSpeed DIGITAL_LEFT
#define ChangeDriveBrake DIGITAL_Y

// =========================
//  SPEED CONSTANTS
// =========================
#define DRIVE_SPEED 60
#define TURN_SPEED 60

// =========================
//  DRIVE SETUP
// =========================
pros::MotorGroup leftDrive({-1, 2, -3});
pros::MotorGroup rightDrive({4, -5, 6});
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Intake motors
pros::Motor motorRoller1(17);
pros::Motor motorRoller2(18);
pros::Motor motorRoller3(19);

// Pneumatics
pros::adi::Pneumatics matchLoader('A', true);
pros::adi::Pneumatics leftArmLoader('B', false);
pros::adi::Pneumatics rightArmLoader('C', false);

// Custom multipliers
float forwardDriveStrength = 1;
float turnDriveStrength = 1;
float motor1Strength = 1;
float motor2Strength = 1;
float motor3Strength = 1;

// =========================
//  EZ-TEMPLATE DRIVE
// =========================
ez::Drive chassis(
    {-1, 2, -3},   // Left motors
    {4, -5, 6},    // Right motors
    7,             // IMU port
    4.125,         // Wheel diameter
    360);          // Motor RPM

// =========================
//  UTILITIES
// =========================
double clamp(double val, double min, double max) {
  if (val > max) return max;
  if (val < min) return min;
  return val;
}

void changeForwardSpeed(double amt) {
  forwardDriveStrength += amt;
  forwardDriveStrength = clamp(forwardDriveStrength, 0.2, 1);
}

void changeTurnSpeed(double amt) {
  turnDriveStrength += amt;
  turnDriveStrength = clamp(turnDriveStrength, 0.2, 1);
}

// =========================
//  INTAKE FUNCTIONS
// =========================
void configure_intake(float first, float second, float third) {
  motorRoller1.move(first * 127 * motor1Strength);
  motorRoller2.move(second * 127 * motor2Strength);
  motorRoller3.move(third * 127 * motor3Strength);
}

void push_up()     { configure_intake(1, 1, -1); }
void push_middle() { configure_intake(1, 1, 1); }
void push_down()   { configure_intake(-1, 1, -1); }
void take_in()     { configure_intake(1, -1, -1); }
void stop_intake() {
  motorRoller1.move(0);
  motorRoller2.move(0);
  motorRoller3.move(0);
}

// =========================
//  INITIALIZE
// =========================
void initialize() {
  ez::ez_template_print();
  pros::delay(500);

  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_curve_default_set(0.3, 0.3);

  chassis.initialize();

  pros::lcd::print(0, "Calibrating IMU...");
  chassis.imu.reset();
  for (int i = 0; i < 2000; i += 20) {
    if (!chassis.imu.is_calibrating()) break;
    pros::delay(20);
  }
  pros::lcd::print(1, "IMU Ready");
}

bool isleft = false;

// =========================
//  AUTONOMOUS ROUTINE
// =========================
//Right
void right_auto() {

  take_in();
  chassis.pid_drive_set(29_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-63_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();
  push_down();
  pros::delay(700);
  stop_intake();
  chassis.pid_drive_set(-38_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(160_deg, -TURN_SPEED);
  chassis.pid_wait();
  matchLoader.set_value(false);
  take_in();
  chassis.pid_drive_set(18_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, DRIVE_SPEED);
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  pros::delay(1500);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  matchLoader.set_value(true);
  stop_intake();
  chassis.pid_turn_set(-20_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, DRIVE_SPEED);
  chassis.pid_wait();
  push_up();
}
void left_auto() {

  take_in();
  chassis.pid_drive_set(29_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(63_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();
  push_middle();
  pros::delay(700);
  stop_intake();
  chassis.pid_drive_set(-38_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-160_deg, -TURN_SPEED);
  chassis.pid_wait();
  matchLoader.set_value(false);
  take_in();
  chassis.pid_drive_set(18_in, DRIVE_SPEED+20);
  chassis.pid_wait();

  chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  chassis.pid_drive_set(5_in, DRIVE_SPEED+20);
  pros::delay(1500);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  matchLoader.set_value(true);
  stop_intake();
  chassis.pid_turn_set(20_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, DRIVE_SPEED);
  chassis.pid_wait();
  push_up();
}


void autonomous(){
  if (isleft){
    left_auto();
  }
  else{
    right_auto();
  }
}

void opcontrol() {
  bool iscoast = true;
  bool matchLoaderVal = false, armLoaderVal = false;

  while (true) {
    // Arcade drive
    float forward = master.get_analog(ANALOG_LEFT_Y) * forwardDriveStrength;
    float turn = master.get_analog(ANALOG_RIGHT_X) * turnDriveStrength;
    leftDrive.move(forward + turn);
    rightDrive.move(forward - turn);

    // Intake buttons
    if (master.get_digital_new_press(PushUpMove)) push_up();
    if (master.get_digital_new_press(PushMiddleMove)) push_middle();
    if (master.get_digital_new_press(PushDownMove)) push_down();
    if (master.get_digital_new_press(TakeInMove)) take_in();
    if (master.get_digital_new_press(StopIntakeMove)) stop_intake();

    // Speed controls
    if (master.get_digital_new_press(IncreaseDriveSpeed)) changeForwardSpeed(0.2);
    if (master.get_digital_new_press(DecreaseDriveSpeed)) changeForwardSpeed(-0.2);
    if (master.get_digital_new_press(IncreaseTurnSpeed)) changeTurnSpeed(0.2);
    if (master.get_digital_new_press(DecreaseTurnSpeed)) changeTurnSpeed(-0.2);
    if (master.get_digital_new_press(ChangeDriveBrake)) iscoast = !iscoast;

    // Pneumatic toggles
    if (master.get_digital_new_press(MatchLoaderAirCompressor))
      matchLoaderVal = !matchLoaderVal;
    if (master.get_digital_new_press(ArmLoaderAirCompressor))
      armLoaderVal = !armLoaderVal;

    // Brake mode
    if (iscoast)
      chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    else
      chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);

    // Pneumatic actions
    matchLoader.set_value(matchLoaderVal);
    leftArmLoader.set_value(armLoaderVal);
    rightArmLoader.set_value(!armLoaderVal);

    pros::delay(20);
  }
}
