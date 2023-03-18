package frc.robot.drive.commands;

import frc.robot.drive.Drive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdDriveAutoBalance extends CommandBase {
	Drive _driveSubsystem;
	double pitchTolerance;
	int facingForward;
	double p;
	double heading;
	double BALANCING_MAX_SPEED;

	public CmdDriveAutoBalance(Drive driveSubsystem) {
		pitchTolerance = 3.0;
		p = 0.07 * BALANCING_MAX_SPEED;
		_driveSubsystem = driveSubsystem;
		BALANCING_MAX_SPEED = Drive.MAX_VELOCITY_METERS_PER_SECOND * 0.25;
		addRequirements(_driveSubsystem);
	}

	// Checks the orientation of the robot, if facing backwards the signs are
	// flipped
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
		double speed = _driveSubsystem.getRobotPitch() * p * facingForward;
		if (Math.abs(speed) >= BALANCING_MAX_SPEED) {
			speed = Math.copySign(BALANCING_MAX_SPEED, speed);
		}
		_driveSubsystem.drive(new ChassisSpeeds(0, speed, 0));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		_driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
	}

	// Returns true when the command should end.
	@Override
	// Ends when pitch = 0
	// TODO: Check that when running this the robot doesn't just flip from one side
	// to the other and the command ends
	public boolean isFinished() {
		if (Math.abs(_driveSubsystem.getRobotPitch()) < pitchTolerance) {
			return true;
		}
		return false;
	}
}
