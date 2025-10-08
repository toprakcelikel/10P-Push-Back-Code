
#include "main.h"


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
