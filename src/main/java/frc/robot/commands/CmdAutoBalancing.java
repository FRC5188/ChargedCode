// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.commands.DefaultDriveCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class CmdAutoBalancing extends CommandBase {
	Drive _driveSubsystem;
	double pitchTolerance;
	int facingForward;
	double p;
	double heading;
	double BALANCING_MAX_SPEED;

	public CmdAutoBalancing(Drive driveSubsystem) {
		pitchTolerance = 3.0;
		p = 0.07 * BALANCING_MAX_SPEED; 
		_driveSubsystem = driveSubsystem;
		BALANCING_MAX_SPEED = _driveSubsystem.MAX_VELOCITY_METERS_PER_SECOND*0.25;
		addRequirements(_driveSubsystem);
	}

	// Checks the orientation of the robot, if facing backwards the signs are flipped
	@Override
	public void initialize() {
		if (-90 < _driveSubsystem.getGyroscopeRotation().getDegrees()
				&& _driveSubsystem.getGyroscopeRotation().getDegrees() < 90) {
			facingForward = 1;
		} else {
			facingForward = -1;
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
  	public void execute() {
		double speed = _driveSubsystem.getRobotPitch()*p*facingForward;
		if(Math.abs(speed) >= BALANCING_MAX_SPEED){
			speed = Math.copySign(BALANCING_MAX_SPEED, speed);
		}
		_driveSubsystem.drive(new ChassisSpeeds(0,speed,0));
  	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		_driveSubsystem.drive(new ChassisSpeeds(0,0,0));
	}

	// Returns true when the command should end.
	@Override
	// Ends when pitch = 0
	// TODO: Check that when running this the robot doesn't just flip from one side to the other and the command ends
	public boolean isFinished() {
		if (-pitchTolerance < _driveSubsystem.getRobotPitch() && _driveSubsystem.getRobotPitch() < pitchTolerance) {
			return true;
		}
		return false;
	}
}
