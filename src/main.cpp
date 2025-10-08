#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "main.h"

pros::MotorGroup left_motors({12, 11}, pros::MotorGearset::green); // left motors on ports 12, 11
pros::MotorGroup right_motors({-20, -19}, pros::MotorGearset::green); // right motors on ports 20, 19

lemlib::Drivetrain driveTrain(&left_motors, &right_motors);

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            
            // delay to save resources
            pros::delay(20);
        }
    });
}

void opcontrol() {
    // loop forever
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        // Axis1 - Left and right of the right joystick.
        // Axis2 - Up and down of the right joystick.
        // Axis3 - Up and down of the left joystick.
        // Axis4 - Left and right of the left joystick.
        // driveTrain.arcadeDrive(Controller.Axis3.value(), Controller.Axis1.value(), Controller.Axis4.value());
  
        // move the robot
        double a3 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double a1 = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        double a4 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // delay to save resources
        pros::delay(25);
    }
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}
