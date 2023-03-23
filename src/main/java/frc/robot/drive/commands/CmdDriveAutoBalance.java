package frc.robot.drive.commands;

import frc.robot.drive.Drive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdDriveAutoBalance extends CommandBase {
	private Drive _driveSubsystem;
	private double BALANCING_MAX_SPEED;
	private double BALANCING_MAX_ACCELERATION;
	private final double PID_TOLERANCE;
	private ProfiledPIDController pidController;
	private TrapezoidProfile.Constraints BALANCING_CONSTRAINTS;

	public CmdDriveAutoBalance(Drive driveSubsystem) {
		_driveSubsystem = driveSubsystem;
		PID_TOLERANCE = 2.0;
		BALANCING_MAX_SPEED = Drive.MAX_VELOCITY_METERS_PER_SECOND * 0.25;
		BALANCING_MAX_ACCELERATION = BALANCING_MAX_SPEED; //TODO: Find the actual max acceleration
		BALANCING_CONSTRAINTS = new TrapezoidProfile.Constraints(BALANCING_MAX_SPEED, BALANCING_MAX_ACCELERATION);
		addRequirements(_driveSubsystem);
		pidController = new ProfiledPIDController(0.02, 0, 0, BALANCING_CONSTRAINTS); //TODO: Tune PID values
	}


	@Override
	public void initialize() {
		pidController.setGoal(0);
		pidController.setTolerance(PID_TOLERANCE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		_driveSubsystem.drive(new ChassisSpeeds(0, pidController.calculate(_driveSubsystem.getRobotPitch()), 0));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		_driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pidController.atGoal();
	}
	
}