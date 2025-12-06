#include "lemlib/api.hpp" // IWYU pragma: keep

#include "liblvgl/llemu.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

#include "main.h"

pros::MotorGroup right_motor_group({18, 19, -20}, pros::MotorGears::blue);
pros::MotorGroup left_motor_group({11, -12, -13}, pros::MotorGears::blue);

pros::Motor IntakeMotor(21);
pros::Motor higherIntakeMotor(2);
pros::Motor midIntakeMotor(10);
pros::adi::Pneumatics mySolenoid('H', false);
pros::adi::Pneumatics Flag('F', false);

lemlib::Drivetrain drivetrain( // left motor group
    &left_motor_group,
    &right_motor_group,       // right motor group
    10.75,                    // 10 inch track width
    lemlib::Omniwheel::NEW_4, // using new 4" omnis
    450,                      // drivetrain rpm is 360
    2                         // horizontal drift is 2 (for now)
);

// imu
pros::IMU imu(-14);

// vertical tracking wheel encoder
pros::Rotation verticalOdom(-16);
pros::Rotation horizontalOdom(17);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&verticalOdom,
                                              lemlib::Omniwheel::NEW_2,
                                              0.5, 
                                              true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalOdom,
                                                lemlib::Omniwheel::NEW_2, 5.75,
                                                true);
// odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1, set to nullptr
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
    &imu     // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(40, // proportional gain (kP) 9
                                              0,  // integral gain (kI) 0
                                              16, // derivative gain (kD) 67
                                     0,  // anti windup
                                      0,  // small error range, in inches
                               0,  // small error range timeout, in milliseconds
                                      0,  // large error range, in inches
                               0, // large error range timeout, in milliseconds
                                            0   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(16, // kP
                                              0,  // kI
                                              21, // kD
                                              3,  // anti windup
                                              1,  // small error range (degrees)
                                              100, // small error timeout (ms)
                                              3, // large error range (degrees)
                                              500, // large error timeout (ms)
                                              0    // max acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                       angular_controller, // angular PID settings
                                        sensors             // odometry sensors
);

void telemetry() {
  while (true) {
    // print robot location to the brain screen
    pros::delay(500);
    pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
    pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
    pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    pros::lcd::print(3, "IMU heading: %f", imu.get_heading());
    // delay to save resources
    pros::delay(20);
  }
}

// initialize function. Runs on program startup
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  pros::delay(500);
  chassis.calibrate(); // calibrate sensors
  // print position to brain screen
  pros::Task screen_task(telemetry);
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

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
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

void intake() {
  IntakeMotor.move(127);
  midIntakeMotor.move(127);
}

void bottomIntake() { IntakeMotor.move(127); }

void putBottom() {
  IntakeMotor.move(-127);
  midIntakeMotor.move(-127);
}

void putHigh() { higherIntakeMotor.move(127); }

void stopAll() {
  higherIntakeMotor.move(0);
  midIntakeMotor.move(0);
  IntakeMotor.move(0);
}

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

void autonomousrightSide() {
  chassis.setPose(7, -50, 0);

  chassis.turnToPoint(24.4, -19.95, 500, {.maxSpeed = 70, .minSpeed = 30});
  bottomIntake();
  chassis.moveToPoint(24.4, -19.95, 4000, {.maxSpeed = 25, .minSpeed = 20});
  pros::delay(500);

  chassis.turnToPoint(11.33, -10.5, 1000, {.maxSpeed = 70, .minSpeed = 30});
  chassis.moveToPoint(11.33, -10.5, 1000, {.maxSpeed = 70, .minSpeed = 30});
  pros::delay(500);
  putBottom();
  pros::delay(1500);

  chassis.moveToPoint(54, -48, 2500, {.forwards = false, .maxSpeed = 60});
  pros::delay(400);
  chassis.turnToHeading(180, 1000, {.maxSpeed = 90});
  pros::delay(600);
  mySolenoid.extend();
  pros::delay(500);
  intake();
  chassis.moveToPoint(58.4, -63.5, 1500, {.maxSpeed = 60, .minSpeed = 20});
  pros::delay(1500);
  chassis.moveToPoint(56, -23, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(1500);
  putHigh();
  pros::delay(1500);
  stopAll();
}

void autonomousSoloOp() {
  pros::delay(1000);
  chassis.setPose(7, -50, 0);
  chassis.turnToPoint(54, -48, 750, {.forwards = false, .maxSpeed = 90});
  chassis.moveToPoint(54, -48, 1500, {.forwards = false, .maxSpeed = 90});
  pros::delay(400);
  chassis.turnToHeading(180, 1000, {.maxSpeed = 90});
  mySolenoid.extend();
  pros::delay(300);
  intake();
  chassis.moveToPoint(58.4, -60.5, 1000, {.maxSpeed = 90, .minSpeed = 20});
  pros::delay(500);
  chassis.moveToPoint(58, -23, 1800, {.forwards = false, .maxSpeed = 90});
  pros::delay(500);
  putHigh();
  pros::delay(600);
  chassis.moveToPoint(58, -48, 1500, {.maxSpeed = 90});
  stopAll();
  mySolenoid.retract();
  pros::delay(500);
  chassis.turnToHeading(240, 1500, {.maxSpeed = 90});
  bottomIntake();
  pros::delay(300);
  chassis.moveToPoint(21, -19.95, 1750, {.maxSpeed = 90, .minSpeed = 20});
  pros::delay(300);
  chassis.moveToPoint(9, -10.5, 1000, {.maxSpeed = 70, .minSpeed = 30});
  putBottom();
  chassis.moveToPoint(21, -19.95, 400,
                      {.forwards = false, .maxSpeed = 70, .minSpeed = 20});
  pros::delay(500);

  pros::delay(1500);
  chassis.turnToPoint(-44, -48, 1000, {.forwards = false, .maxSpeed = 90});
  chassis.moveToPoint(-44, -48, 2000, {.forwards = false, .maxSpeed = 90});
  pros::delay(400);
  chassis.turnToHeading(180, 1000, {.maxSpeed = 90});
  pros::delay(400);
  mySolenoid.extend();
  pros::delay(500);
  intake();
  chassis.moveToPoint(-51.4, -60.5, 1000, {.maxSpeed = 80, .minSpeed = 20});
  pros::delay(1500);
  chassis.moveToPoint(58, -23, 1800, {.forwards = false, .maxSpeed = 90});
  pros::delay(500);
  putHigh();
  pros::delay(1000);
  chassis.moveToPoint(58, -48, 1500, {.maxSpeed = 90});
  stopAll();
}

void newautonomousSoloOp() {
  chassis.setPose(-53.3, 12, 0);
  chassis.turnToPoint(-53, -46, 750, {.forwards = false, .maxSpeed = 90});
  chassis.moveToPoint(-53, -46, 1500, {.forwards = false, .maxSpeed = 90});
  pros::delay(400);
  chassis.turnToHeading(180, 1000, {.maxSpeed = 90});
  mySolenoid.extend();
  pros::delay(300);
  intake();
  chassis.moveToPoint(-65, 46, 1000, {.maxSpeed = 90, .minSpeed = 20});
  pros::delay(500);
  chassis.moveToPoint(-25, 46, 1800, {.forwards = false, .maxSpeed = 90});
  pros::delay(500);
  putHigh();
  pros::delay(600);
  stopAll();
  mySolenoid.retract();
}

void autonomous() {

  chassis.setPose(0, 0, 0, 0);
  chassis.turnToHeading(90, 2000, {.maxSpeed = 80});
  // chassis.moveToPoint(0,25,2000, {.maxSpeed=60});
}

void opcontrol() {
  // autonomous();
  //  loop forever
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  bool pistonState = false;
  bool flagState = false;
  while (true) {

    // get left y and right y 3
    float forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    float heading = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.99;

    // move the robot
    chassis.arcade((int) forward, (int) heading);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      IntakeMotor.move(127);
      midIntakeMotor.move(-127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      IntakeMotor.move(-127);
      midIntakeMotor.move(127);
    } else {
      IntakeMotor.move(0);
      midIntakeMotor.move(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      higherIntakeMotor.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      higherIntakeMotor.move(-127);
    } else {
      higherIntakeMotor.move(0);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      if (pistonState == false) {
        mySolenoid.extend();
        pistonState = true;
      } else {
        mySolenoid.retract();
        pistonState = false;
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      if (flagState == false) {
        Flag.extend();
        flagState = true;
      } else {
        Flag.retract();
        flagState = false;
      }
    }

    // delay to save resources
    pros::delay(10);
  }
}
