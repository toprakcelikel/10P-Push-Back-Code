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

  MecanumDriveTrain(): frontLeft(12),
                       backLeft(11), 
                       frontRight(-20),
                       backRight(-19){}


  void arcadeDrive(double a3, double a1, double a4) {
    frontLeftVelocity  = (a3 + a1 + a4) / 2;
    backLeftVelocity   = (a3 + a1 - a4) / 2; 
    frontRightVelocity = (a3 - a1 - a4) / 2; 
    backRightVelocity  = (a3 - a1 + a4) / 2;

    frontLeft.move_velocity(frontLeftVelocity);
    backLeft.move_velocity(backLeftVelocity);
    frontRight.move_velocity(frontRightVelocity);
    backRight.move_velocity(backRightVelocity);
  }

private: 
  pros::Motor frontLeft;
  pros::Motor frontRight;

  pros::Motor backLeft;
  pros::Motor backRight;
};
#endif