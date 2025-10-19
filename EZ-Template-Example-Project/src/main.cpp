#include "main.h"


#define PushUpMove DIGITAL_R1
#define PushMiddleMove DIGITAL_R2
#define PushDownMove DIGITAL_L1
#define TakeInMove DIGITAL_L2
#define StopIntakeMove DIGITAL_B
#define ArmLoaderAirCompressor DIGITAL_A
#define MatchLoaderAirCompressor DIGITAL_B

// Left and right drive motors (use your actual ports)
pros::MotorGroup leftDrive({-1, 2, -3});
pros::MotorGroup rightDrive({4, -5, 6});

// roller motors
pros::Motor motorRoller1(17);
pros::Motor motorRoller2(18);
pros::Motor motorRoller3(19);
pros::Motor motorRoller4(20);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::Pneumatics matchLoader('A', false);
pros::adi::Pneumatics armLoader('B', false);

// Your drive setup
ez::Drive chassis(
    {-1, 2, -3},     // Left motors
    {4, -5, 6},      // Right motors
    7,               // IMU Port
    4.125,           // Wheel Diameter
    360);            // Wheel RPM

void initialize() {
  ez::ez_template_print();
  pros::delay(500);

  // Configure your chassis
  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); //Coast-Brake-Hold
  chassis.opcontrol_curve_default_set(0.3, 0.3); //Format: (throttle curve, turn curve)

  // Initialize chassis only (no auton selector)
  chassis.initialize();

  // Roller init (start forward)
  motorRoller1.move(127);
  motorRoller2.move(127);
  motorRoller3.move(127);
  motorRoller4.move(127);
}

/*
void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  // Call your short 15s autonomous routine here
}
*/

void configure_intake(bool first_motor, bool second_motor, bool third_motor, bool fourth_motor){
    motorRoller1.move(first_motor ? -127 : 127);
    motorRoller2.move(second_motor ? -127 : 127);
    motorRoller3.move(third_motor ? -127 : 127);
    motorRoller4.move(fourth_motor ? -127 : 127);
}

void push_up(){
  configure_intake(true, true, false, true); //down up left
}
void push_middle(){
  configure_intake(true, true, true, false); //down up right
}
void push_down(){
  configure_intake(false, true, false, false); //down
}
void take_in(){
  configure_intake(true, false, false, false);
}
void stop_intake(){
    motorRoller1.move(0);
    motorRoller2.move(0);
    motorRoller3.move(0);
    motorRoller4.move(0);

}

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  bool matchLoaderValue = false, armLoaderValue = false;

  while (true) {
    // Arcade drive control
    int forward = master.get_analog(ANALOG_LEFT_Y);   // Forward / backward
    int turn = master.get_analog(ANALOG_RIGHT_X);     // Turning

    int leftPower = forward + turn;
    int rightPower = forward - turn;

    leftDrive.move(leftPower);
    rightDrive.move(rightPower);

    if (master.get_digital_new_press(PushUpMove)) push_up();
    if (master.get_digital_new_press(PushMiddleMove)) push_middle();
    if (master.get_digital_new_press(PushDownMove)) push_down();
    if (master.get_digital_new_press(TakeInMove)) take_in();
    if (master.get_digital_new_press(StopIntakeMove)) stop_intake();
    if (master.get_digital_new_press(MatchLoaderAirCompressor))matchLoaderValue = !matchLoaderValue;
    if (master.get_digital_new_press(ArmLoaderAirCompressor))armLoaderValue = !armLoaderValue;
    matchLoader.set_value(matchLoaderValue);
    armLoader.set_value(armLoaderValue);

    pros::delay(ez::util::DELAY_TIME);
  }
}
