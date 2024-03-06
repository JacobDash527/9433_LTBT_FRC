// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <math.h>
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
#include <frc/TimedRobot.h>


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
	std::cout << "-- LTBT Robot Program Start --" << std::endl;

	cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
	ahrs->Reset();
	camera.SetResolution(640, 480);
	cs::CvSink cvSink = frc::CameraServer::GetVideo();
	cs::CvSource outputStream = frc::CameraServer::PutVideo("Video", 640, 480);
	
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
	

	double joyXPower = controller2.GetRawAxis(1) * fabs(controller2.GetRawAxis(1)); 
	double joyYPower = controller2.GetRawAxis(0) * fabs(controller2.GetRawAxis(0));
	double joyZPower = controller2.GetRawAxis(4);

	// double joyYPower = controller2.GetRawAxis(1);
	// double joyZPower = controller2.GetRawAxis(4);
	// double joyXPower = controller2.GetRawAxis(0); 

	// if (fabs(joyYPower) > 0.1)
	// {
	// 	joyXPower = joystick.GetX() * fabs(joystick.GetX());
	// }
	// else
	// {
	// 	joyXPower = 0;
	// }

	

	//std::cout << "GetAngle:" << ahrs->GetAngle() << "\n";
	// std::cout << "GetRads:" << ahrs->GetAngle() * (M_PI / 180) << "\n"; 
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
	double fieldVelocityHeading;
	if (joyXPower < 0) {
	fieldVelocityHeading = std::atan(joyXPower / joyYPower);
	}
	else {
	  fieldVelocityHeading = std::atan(joyXPower / joyYPower) + M_PI;
	}

	double robotVelocityHeading = YawRads + fieldVelocityHeading;

	double RobotSpeed = sqrt(pow(joyXPower, 2) + pow(joyYPower, 2));


	double x_rotated = cos(robotVelocityHeading) * RobotSpeed;
	double y_rotated = sin(robotVelocityHeading) * RobotSpeed;

	double motors [4] = {0,0,0,0};

	// if (std::abs(joystick.GetX()) > 0.15 )
	// {
		// if going left, spin left wheels outer from eachother, spin right inner
		motors[0] += (x_rotated);
		motors[1] += (-x_rotated);

		motors[2] += (-x_rotated);
		motors[3] += (x_rotated);
	// }

	// if (std::abs(joystick.GetY()) > 0.2 )
	// {
		// left
		motors[0] += (y_rotated);
		motors[1] += (y_rotated);

		// right
		motors[2] += (-y_rotated);
		motors[3] += (-y_rotated);
	// }

	// if (std::abs(joystick.GetZ()) > 0.4 )
	// {
		// left
		motors[0] -= (joyZPower);
		motors[1] -= (joyZPower);

		// right
		motors[2] -= (joyZPower);
		motors[3] -= (joyZPower);
	// }


	frontL.Set(motors[0]);
	backL.Set(motors[1]);

	backR.Set(motors[2]);
	frontR.Set(motors[3]);

	// Create new arm object
	double _leftJoy = -controller.GetRawAxis(1); 

    double leftJoy = filter.Calculate(_leftJoy); 

	double _rightJoy = controller.GetRawAxis(5);
	int _bButton = controller.GetRawButton(2);

	// if (_bButton) {
	// 	arm.Set(-0.1);
	// }
	// else {
	// 	arm.Set(0);
	// }


    double leftPower = leftJoy;
    double rightPower = _rightJoy * fabs(_rightJoy);

	if (_rightJoy >= 0.1){
		arm.Set(_rightJoy*_rightJoy*0.1);
	}
	else if (_rightJoy <= -0.1){
		arm.Set(_rightJoy*_rightJoy*-0.4);
	}
	else {
		arm.Set(0);
	}

	

	// if (_bButton)
	// {
	// 	shooter.Set(-1);
	// 	// std::cout << "SHOOT!" << "\n"; 
	// }
	// else
	// {
	// 	shooter.Set(0);
	// }
	int _yButton = controller.GetRawButton(4);
	int _aButton = controller.GetRawButton(1);

	if (_yButton)
	{
		climber.Set(0.4);
	}
	else if (_aButton)
	{
		climber.Set(-0.25);
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


	// int _lBumper = controller.GetRawButton(5);
	// int _rBumper = controller.GetRawButton(6);

    if (_leftJoy >= 0.1)
    {
        armSpeed = _leftJoy*_leftJoy*1;
		intake.Set(armSpeed);
     
    }
    else if (_leftJoy <= -0.1)
    {
        armSpeed = _leftJoy*_leftJoy*0.6;
		intake.Set(-armSpeed);
       
    }
    else
    {
        intake.Set(0);
     
    }

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
