#include "Drive.h"
#include <frc/filter/SlewRateLimiter.h>
#include <thread>
#include <chrono>

Drive::Drive(double deadZone, double maxSpeed)
{
	// assign arguments to private variables (hance the underscores)
	_deadZone = deadZone;
	_maxSpeed = maxSpeed;

	// Set DeadZones for MecanumDrive and DriveTrain	
	mec_drive.SetDeadband(deadZone);
	// tank_drive.SetDeadband(deadZone);
	
	// Set MaxOutput for MecanumDrive and DriveTrain
	mec_drive.SetMaxOutput(maxSpeed);
	// tank_drive.SetMaxOutput(maxSpeed);
	

}

void Drive::MecDrive(double joyX, double joyY, double joyZ, double fieldOrient)
{

	
	double x_rotated = joyX * cos(fieldOrient) - joyY * sin(fieldOrient);
	double y_rotated = joyX * sin(fieldOrient) + joyY * cos(fieldOrient);

	double joyYPower = y_rotated * fabs(y_rotated);
	double joyXPower = x_rotated * fabs(x_rotated);
	double joyZPower = joyZ * fabs(joyZ);

	double motors [4] = {0,0,0,0};

	if (std::abs(joyX) > 0.15 )
	{
		// if going left, spin left wheels outer from eachother, spin right inner
		motors[0] += (-joyXPower * 0.8);
		motors[1] += (joyXPower * 0.8);

		motors[2] += (joyXPower * 0.8);
		motors[3] += (-joyXPower * 0.8);
	}

	if (std::abs(joyY) > 0.2 )
	{
		// left
		motors[0] += (joyYPower);
		motors[1] += (joyYPower);

		// right
		motors[2] += (-joyYPower);
		motors[3] += (-joyYPower);
	}

	if (std::abs(joyZ) > 0.4 )
	{
		// left
		motors[0] -= (joyZPower * 0.65);
		motors[1] -= (joyZPower * 0.65);

		// Right
		motors[2] -= (joyZPower * 0.65);
		motors[3] -= (joyZPower * 0.65);

	}

	frontL.Set(motors[0]);
	backL.Set(motors[1]);

	backR.Set(motors[2]);
	frontR.Set(motors[3]);

}


void Drive::TrainDrive()
{
	// assign joystick values
	// double leftJoy = -controller.GetRawAxis(1); 
	// double rightJoy = controller.GetRawAxis(5);
	
	// // left motor speed, right motor speed, square joystick intensity true/false
	// tank_drive.TankDrive(leftJoy, rightJoy, true);	
}

/// RAW FUNCTIONS

void Drive::RawMecDrive(double speedX, double speedY, double rotationZ)
{
	mec_drive.DriveCartesian(speedX, speedY, rotationZ);
}

void Drive::RawTrainDrive(double speedL, double speedR)
{
	// tank_drive.TankDrive(speedL, speedR, false);
}

void Drive::StopDrives()
{
	// tank_drive.StopMotor();
	mec_drive.StopMotor();
}

void Drive::SetDriveSafety(bool enabled)
{
	// tank_drive.SetSafetyEnabled(enabled);
	mec_drive.SetSafetyEnabled(enabled);
}