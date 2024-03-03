// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <math.h>
#include <algorithm>
#include "Robot.h"
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include "Drive.h"
#include "Auto.h"
#include "Arm.h"
#include <frc/filter/SlewRateLimiter.h>
#include <thread>
#include <chrono>
#include "cameraserver/CameraServer.h"
#include <frc/AnalogGyro.h>


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
	std::cout << "-- LTBT Robot Program Start --" << std::endl;

	ahrs->Calibrate();
	ahrs->Reset();

	

	// frc::CameraServer::StartAutomaticCapture();

	// cs::CvSink cvSink = frc::CameraServer::GetVideo();

	// cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur", 640, 480);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
	// Auto newAuto;

	//newAuto.TimedAutoArmBendTwo(2000, -0.4);

	//newAuto.TimedAutoIntake(2000, 0.5);

	//newAuto.ArmStop();

	//newAuto.TimedAutoMecDrive(10000, 0.0, -0.25, 0.0);
    using namespace std::this_thread;
	using namespace std::chrono;
	// set motors fo move backwards
	backL.Set(autoSpeed);
	backR.Set(autoSpeed);
	frontL.Set(autoSpeed);
	frontR.Set(autoSpeed);
	// sleep for 5 seconds
	sleep_for(milliseconds(5000));
	// set motors to stop
	backL.Set(0);
	backR.Set(0);
	frontL.Set(0);
	frontR.Set(0);
}

void Robot::AutonomousPeriodic() 
{
	// Auto newAuto;
	
	
	// if (latch == true) 
	// {	
	// 	newAuto.TimedAutoMecDrive(1000, 0, 0.1, 0, 0);

	// 	latch = false;

	// }

}

void Robot::TeleopInit() 
{
	ahrs->Reset();
}

void Robot::TeleopPeriodic() 
{
	// Note for Teleop, DONT USE FOR LOOPS OR WHILE LOOPS! (unless it is really fast / no sleep_for() in the loop) 
	// it will stay at the loop until it is completed (essentially freezing your robot).
	// Teleop is already called every cycle, so use this to your advantage.

	//backL.Set(0.5);

	using namespace frc;
	// create drive object	
	// // DeadZone, MaxSpeed
	//Drive newMec(0.02, 0.8);
	double joyYPower = joystick.GetY() * fabs(joystick.GetY());
	double joyZPower = joystick.GetZ() * fabs(joystick.GetZ());
	double joyXPower = joystick.GetX() * fabs(joystick.GetX());
	//double joySliderPower = 1 - ((joystick.GetRawAxis(4) + 1) / 2);

	// if (fabs(joyYPower) > 0.1)
	// {
	// 	joyXPower = joystick.GetX() * fabs(joystick.GetX());
	// }
	// else
	// {
	// 	joyXPower = 0;
	// }

	

	//std::cout << "GetAngle:" << ahrs->GetAngle() << "\n";
	std::cout << "GetRads:" << ahrs->GetAngle() * (M_PI / 180) << "\n"; 
	// std::cout << "GetDisX:" << ahrs->GetDisplacementX() << "\n";
	// std::cout << "GetDisY:" << ahrs->GetDisplacementY() << "\n";
	
	// std::cout << "GetRate:" << ahrs->GetRate() << "\n"; 
	// std::cout << "GetOffset:" << m_gyro.GetOffset() << "\n"; 
	
	double YawRads = ahrs->GetAngle() * (M_PI / 180);
	// double YawX = cos(YawRads);
	// double YawY = sin(YawRads);

	// frc::Rotation2d YawFinal = Rotation2d(YawX, YawY);

	//mec_drive.DriveCartesian(joyZPower * speed, joyXPower * speed, joyYPower * speed);
	//Wait(0.005_s); // wait 5ms to avoid hogging CPU cycles
 // Invert stick Y axis
	// double x_rotated = joyXPower * cos(YawRads) - joyYPower * sin(YawRads);
	// double y_rotated = joyXPower * sin(YawRads) + joyYPower * cos(YawRads);
	
	double x_rotated = joyXPower * 1.2;
	double y_rotated = joyYPower * 1.2;
	
	double translationAngle = atan2(joystick.GetX(), joystick.GetRawAxis(0));
    double translationPower = hypot(joystick.GetRawAxis(0), joystick.GetRawAxis(1));
    double turnPower = joystick.GetRawAxis(2);
        // use field centric controls by subtracting off the robot angle
	translationAngle -= YawRads;
	
	
	// calculate motor power
    double ADPower = translationPower * sqrt(2) * 0.5 * (sin(translationAngle) + cos(translationAngle));
    double BCPower = translationPower * sqrt(2) * 0.5 * (sin(translationAngle) - cos(translationAngle));

    // check if turning power will interfere with normal translation
    // check ADPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
    double turningScale = std::max(abs(ADPower + turnPower), std::abs(ADPower - turnPower));
    // check BCPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
    turningScale = std::max(turningScale, std::max(std::abs(BCPower + turnPower), std::abs(BCPower - turnPower)));

    // adjust turn power scale correctly
    if (std::abs(turningScale) < 1.0)
    {
        turningScale = 1.0;
    }

    // set the motors, and divide them by turningScale to make sure none of them go over the top, which would alter the translation angle
    frontL.Set((ADPower - turningScale) / turningScale);
    backL.Set((BCPower - turningScale) / turningScale);
    frontR.Set((BCPower + turningScale) / turningScale);
    backR.Set((ADPower + turningScale) / turningScale);


		// right
	// frontL.Set(motors[0] * speed);
	// backL.Set(motors[1] * speed);

	// backR.Set(motors[2] * speed);
	// frontR.Set(motors[3] * speed);

	// Create new arm object
	double _leftJoy = -controller.GetRawAxis(1); 

    double leftJoy = filter.Calculate(_leftJoy); 

	double _rightJoy = controller.GetRawAxis(5);
	int _bButton = controller.GetRawButton(2);


    double leftPower = leftJoy;
    double rightPower = _rightJoy * fabs(_rightJoy);

	if (_bButton)
	{
		shooter.Set(-1);
		// std::cout << "SHOOT!" << "\n"; 
	}
	else
	{
		shooter.Set(0);
	}
	int _yButton = controller.GetRawButton(4);
	int _aButton = controller.GetRawButton(1);

	if (_yButton)
	{
		climber.Set(0.2);
	}
	else if (_aButton)
	{
		climber.Set(-0.2);
	}
	else
	{
		climber.Set(0);
	}
    
    // if (_bButton)
    // {
    //     bendOne.Set(0);
	// 	bendTwo.Set(0);
    // }
    // else if (joyYPower < 0.7)
    // {
    //     bendOne.Set(-rightPower * maxSpeed);
    //     bendTwo.Set(-leftPower * maxSpeed);
    // }
	// else if (joyYPower > 0.7)
	// {
	// 	bendOne.Set(ControlMode::PercentOutput,-0.1);
	// 	bendTwo.Set(ControlMode::PercentOutput,0.1); 
	//}


	int _lBumper = controller.GetRawButton(5);
	int _rBumper = controller.GetRawButton(6);
	

    // if (_rBumper)
    // {
    //     speed = 0.5;
	// 	intake1.Set(speed);
    //     intake2.Set(-speed);
    // }
    // else if (_lBumper)
    // {
    //     speed = 0.8;
	// 	intake1.Set(-speed);
    //     intake2.Set(speed);
    // }
    // else
    // {
    //     intake1.Set(0);
    //     intake2.Set(0);
    // }

}


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
