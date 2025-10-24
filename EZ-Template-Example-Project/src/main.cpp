#include "main.h"
#include <cmath>

// ====================
// CONTROLLER & BUTTONS
// ====================
#define PushUpMove DIGITAL_R1
#define PushMiddleMove DIGITAL_R2
#define PushDownMove DIGITAL_L1
#define TakeInMove DIGITAL_L2
#define StopIntakeMove DIGITAL_X
#define ArmLoaderAirCompressor DIGITAL_A
#define MatchLoaderAirCompressor DIGITAL_B

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ====================
// MOTORS & SENSORS
// ====================
pros::MotorGroup leftDrive({-1, 2, -3});
pros::MotorGroup rightDrive({4, -5, 6});

pros::Motor motorRoller1(17);
pros::Motor motorRoller2(18);
pros::Motor motorRoller3(19);
pros::Motor motorRoller4(20);

pros::adi::Pneumatics matchLoader('A', false);
pros::adi::Pneumatics armLoader('B', false);

pros::Imu imu(7);
pros::Rotation rotationSensor(8); // adjust to your port

// ====================
// PID CONSTANTS
// ====================
double kP_drive = 0.4;
double kD_drive = 0.2;
double kP_turn = 0.5;
double kD_turn = 0.3;

// ====================
// SENSOR HELPERS
// ====================
void resetSensors() {
  imu.tare();
  rotationSensor.reset();
}

double getHeading() {
  return imu.get_rotation();
}

double getDistance() {
  double wheelDiameter = 4.125;
  double wheelCircumference = wheelDiameter * M_PI;
  return (rotationSensor.get_angle() / 360.0) * wheelCircumference;
}

// ====================
// DRIVE & TURN PID
// ====================
void movePID(double targetInches, int maxSpeed = 100) {
  resetSensors();
  double prevError = 0;
  double error = 0;
  double power = 0;

  while (true) {
    double current = getDistance();
    error = targetInches - current;
    double derivative = error - prevError;
    power = kP_drive * error + kD_drive * derivative;

    // Clamp output
    if (power > maxSpeed) power = maxSpeed;
    if (power < -maxSpeed) power = -maxSpeed;

    leftDrive.move(power);
    rightDrive.move(power);

    if (fabs(error) < 0.5) break; // within 0.5 inch tolerance
    prevError = error;
    pros::delay(20);
  }

  leftDrive.brake();
  rightDrive.brake();
}

void turnPID(double targetDeg, int maxSpeed = 100) {
  imu.tare();
  double prevError = 0;
  double error = 0;
  double power = 0;

  while (true) {
    double heading = imu.get_rotation();
    error = targetDeg - heading;
    double derivative = error - prevError;
    power = kP_turn * error + kD_turn * derivative;

    if (power > maxSpeed) power = maxSpeed;
    if (power < -maxSpeed) power = -maxSpeed;

    leftDrive.move(power);
    rightDrive.move(-power);

    if (fabs(error) < 1.0) break;
    prevError = error;
    pros::delay(20);
  }

  leftDrive.brake();
  rightDrive.brake();
}

// ====================
// INTAKE CONTROL
// ====================
void configure_intake(bool first_motor, bool second_motor, bool third_motor, bool fourth_motor, bool stop_fourth) {
  motorRoller1.move(first_motor ? -127 : 127);
  motorRoller2.move(second_motor ? -127 : 127);
  motorRoller3.move(third_motor ? -127 : 127);
  motorRoller4.move(fourth_motor ? -127 : 127);
  if (stop_fourth) motorRoller4.move(0);
}

void push_up() { configure_intake(true, true, false, true, false); }
void push_middle() { configure_intake(true, true, true, false, true); }
void push_down() { configure_intake(false, true, false, false, false); }
void take_in() { configure_intake(true, false, false, false, true); }
void stop_intake() {
  motorRoller1.move(0);
  motorRoller2.move(0);
  motorRoller3.move(0);
  motorRoller4.move(0);
}

// ====================
// AUTONOMOUS ROUTINES
// ====================
int autonPosition = 0; // 0 = left, 1 = right

void autonRight() {
  pros::lcd::print(0, "Running RIGHT auton");

  movePID(24);
  turnPID(-45);
  movePID(20);
  turnPID(45);
  movePID(-12);
}

void autonLeft() {
  pros::lcd::print(0, "Running LEFT auton");

  movePID(24);
  turnPID(45);
  movePID(20);
  turnPID(-45);
  movePID(-12);
}

// ====================
// COMPETITION FUNCTIONS
// ====================
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Initializing...");
  imu.reset();
  while (imu.is_calibrating()) {
    pros::delay(100);
  }
  pros::lcd::set_text(1, "IMU Ready!");
  pros::delay(200);
}

void competition_initialize() {
  pros::lcd::set_text(0, "Press LEFT for Left Auton");
  pros::lcd::set_text(1, "Press RIGHT for Right Auton");

  while (true) {
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      autonPosition = 0;
      pros::lcd::set_text(2, "Selected: LEFT auton");
    }
    if (master.get_digital_new_press(DIGITAL_RIGHT)) {
      autonPosition = 1;
      pros::lcd::set_text(2, "Selected: RIGHT auton");
    }
    pros::delay(20);
  }
}

void autonomous() {
  if (autonPosition == 1) autonRight();
  else autonLeft();
}

// ====================
// DRIVER CONTROL
// ====================
void opcontrol() {
  bool matchLoaderValue = false, armLoaderValue = false;

  while (true) {
    int forward = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);

    int leftPower = forward + turn;
    int rightPower = forward - turn;

    leftDrive.move(leftPower);
    rightDrive.move(rightPower);

    if (master.get_digital_new_press(PushUpMove)) push_up();
    if (master.get_digital_new_press(PushMiddleMove)) push_middle();
    if (master.get_digital_new_press(PushDownMove)) push_down();
    if (master.get_digital_new_press(TakeInMove)) take_in();
    if (master.get_digital_new_press(StopIntakeMove)) stop_intake();

    if (master.get_digital_new_press(MatchLoaderAirCompressor))
      matchLoaderValue = !matchLoaderValue;
    if (master.get_digital_new_press(ArmLoaderAirCompressor))
      armLoaderValue = !armLoaderValue;

    matchLoader.set_value(matchLoaderValue);
    armLoader.set_value(armLoaderValue);

    pros::delay(20);
  }
}
