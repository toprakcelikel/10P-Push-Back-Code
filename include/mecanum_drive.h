#ifndef ___MECANUM_DRIVE____
#define ___MECANUM_DRIVE____

#define INCH2METERS 25.4/1000

#include "lemlib/api.hpp" // IWYU pragma: keep

class MecanumDriveTrain {
public: 
  double WheelDiameter = 2.0* INCH2METERS; 
  double TrackWidth = 11.5 * INCH2METERS; 
  double WheelBase = 6.5*INCH2METERS;

  double frontLeftVelocity;
  double backLeftVelocity;
  double frontRightVelocity;
  double backRightVelocity;

  MecanumDriveTrain(): frontLeftMotor(12),
                       backLeftMotor(11), 
                       frontRightMotor(-20),
                       backRightMotor(-19){}


  void arcadeDrive(double a3, double a1, double a4) {
    frontLeftVelocity  = (a3 + a1 + a4) / 2;
    backLeftVelocity   = (a3 + a1 - a4) / 2; 
    frontRightVelocity = (a3 - a1 - a4) / 2; 
    backRightVelocity  = (a3 - a1 + a4) / 2;

    frontLeftMotor.move_velocity(frontLeftVelocity);
    backLeftMotor.move_velocity(backLeftVelocity);
    frontRightMotor.move_velocity(frontRightVelocity);
    backRightMotor.move_velocity(backRightVelocity);
  }

private: 
  pros::Motor frontLeftMotor;
  pros::Motor frontRightMotor;

  pros::Motor backLeftMotor;
  pros::Motor backRightMotor;
};
#endif