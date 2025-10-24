#include "main.h"

//  BUTTON & DEVICE DEFINES
#define PushUpMove DIGITAL_R1
#define PushMiddleMove DIGITAL_R2
#define PushDownMove DIGITAL_L1
#define TakeInMove DIGITAL_L2
#define StopIntakeMove DIGITAL_X
#define ArmLoaderAirCompressor DIGITAL_A
#define MatchLoaderAirCompressor DIGITAL_B

//  DRIVE SETUP
pros::MotorGroup leftDrive({-1, 2, -3});
pros::MotorGroup rightDrive({4, -5, 6});
pros::Controller master(pros::E_CONTROLLER_MASTER);


pros::Motor motorRoller1(17);
pros::Motor motorRoller2(18);
pros::Motor motorRoller3(19);

// Pneumatics
pros::adi::Pneumatics matchLoader('A', false);
pros::adi::Pneumatics armLoader('B', false);

// EZ-Template drive
ez::Drive chassis(
    {-1, 2, -3},
    {4, -5, 6},
    7,        // IMU
    4.15,    // wheel diameter
    360);     // wheel rpm

// Rotation sensors
pros::Rotation leftRot(8);
pros::Rotation rightRot(9);

//  PID CONSTANTS
double kP_drive = 0.45;
double kI_drive = 0.0;
double kD_drive = 3.0;

double kP_turn = 2.2;
double kI_turn = 0.0;
double kD_turn = 0.9;

// Utility clamp function (no std::clamp)
double clamp(double val, double min, double max) {
  if (val > max) return max;
  if (val < min) return min;
  return val;
}

//  INITIALIZE
void initialize() {
  ez::ez_template_print();
  pros::delay(500);

  chassis.initialize();
  pros::lcd::print(0, "Calibrating IMU...");
  chassis.imu.reset();
  pros::delay(2000);
  pros::lcd::print(1, "IMU Ready");

  leftRot.reset_position();
  rightRot.reset_position();
}

//  INTAKE FUNCTIONS
void configure_intake(int first, int second, int third) {
  motorRoller1.move(first*127);
  motorRoller2.move(second*127);
  motorRoller3.move(third*127);
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
//  DRIVE PID
// =========================

void drivePID(double targetInches, int maxSpeed = 100) {
  leftRot.reset_position();
  rightRot.reset_position();

  double wheelDiameter = 4.125;
  double wheelCircumference = wheelDiameter * M_PI;
  double degPerInch = 360.0 / wheelCircumference;
  double targetDeg = targetInches * degPerInch;

  double error = 0, prevError = 0, derivative = 0, integral = 0;
  double leftPos = 0, rightPos = 0, avgPos = 0;
  double headingError = 0;
  double headingStart = chassis.imu.get_rotation();

  while (true) {
    leftPos = leftRot.get_position();
    rightPos = rightRot.get_position();
    avgPos = (leftPos + rightPos) / 2.0;

    error = targetDeg - avgPos;
    derivative = error - prevError;
    integral += error;
    prevError = error;

    double headingNow = chassis.imu.get_rotation();
    headingError = headingNow - headingStart;

    double power = kP_drive * error + kI_drive * integral + kD_drive * derivative;
    power = clamp(power, -maxSpeed, maxSpeed);

    double correction = headingError * 1.5;
    double leftPower = power - correction;
    double rightPower = power + correction;

    leftDrive.move(leftPower);
    rightDrive.move(rightPower);

    if (fabs(error) < 15) break;
    pros::delay(10);
  }
  leftDrive.move(0);
  rightDrive.move(0);
}

//  TURN PID
void turnPID(double targetAngle, int maxSpeed = 80) {
  double error = 0, prevError = 0, derivative = 0, integral = 0;

  while (true) {
    double current = chassis.imu.get_rotation();
    error = targetAngle - current;
    derivative = error - prevError;
    integral += error;
    prevError = error;

    double power = kP_turn * error + kI_turn * integral + kD_turn * derivative;
    power = clamp(power, -maxSpeed, maxSpeed);

    leftDrive.move(-power);
    rightDrive.move(power);

    if (fabs(error) < 1.5) break;
    pros::delay(10);
  }
  leftDrive.move(0);
  rightDrive.move(0);
}

//  AUTONOMOUS
//void autonomous() {
//  chassis.drive_sensor_reset();
//  pros::lcd::print(0, "Running PID auton...");
//
//  drivePID(24);     // drive forward 24 inches
//  pros::delay(300);
//  turnPID(90);      // turn 90 deg
//  pros::delay(300);
//  drivePID(-12);    // back 12 inches
//  pros::delay(300);
//  turnPID(0);       // return to heading 0
//}

//  DRIVER CONTROL
void opcontrol() {
  bool matchLoaderVal = false, armLoaderVal = false;

  while (true) {
    // Arcade drive
    int forward = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    leftDrive.move(forward + turn);
    rightDrive.move(forward - turn);

    // Intake buttons
    if (master.get_digital_new_press(PushUpMove)) push_up();
    if (master.get_digital_new_press(PushMiddleMove)) push_middle();
    if (master.get_digital_new_press(PushDownMove)) push_down();
    if (master.get_digital_new_press(TakeInMove)) take_in();
    if (master.get_digital_new_press(StopIntakeMove)) stop_intake();

    // Pneumatic toggles
    if (master.get_digital_new_press(MatchLoaderAirCompressor))
      matchLoaderVal = !matchLoaderVal;
    if (master.get_digital_new_press(ArmLoaderAirCompressor))
      armLoaderVal = !armLoaderVal;

    matchLoader.set_value(matchLoaderVal);
    armLoader.set_value(armLoaderVal);

    pros::delay(20);
  }
}
