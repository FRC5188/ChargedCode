package frc.robot.drive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drive;

import java.util.function.DoubleSupplier;

public class CmdDriveAutoRotate extends CommandBase {
    private final Drive m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final double m_angleSetpoint;
    private ProfiledPIDController m_angleController;
    private TrapezoidProfile.Constraints m_constraints;

    // windham gains were 0.02, 0.0, 0.0. It worked well for their week 1
    private final double AUTO_ROTATE_KP = 0.2;
    private final double AUTO_ROTATE_KI = 0.001;
    private final double AUTO_ROTATE_KD = 0.001;
    private final double AUTO_ROTATE_TOLERANCE = 1.0;

    /**
     * Auto Rotate the robot to a specified angle. This command will still allow the
     * robot to drive while it rotates.
     * This command takes away rotation control from the driver until its finished.
     * 
     * @param drivetrainSubsystem  the drive subsytem
     * @param translationXSupplier a double supplier for the x movement
     * @param translationYSupplier a souble supplier for the y movement
     * @param angleSetpoint        the target angle of the robot heading
     */
    public CmdDriveAutoRotate(Drive drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            double angleSetpoint) {

        this.m_constraints = new TrapezoidProfile.Constraints(40, 40);

        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_angleController = new ProfiledPIDController(this.AUTO_ROTATE_KP, this.AUTO_ROTATE_KI,
                this.AUTO_ROTATE_KD, m_constraints);
        this.m_angleSetpoint = angleSetpoint;
        this.m_angleController.setTolerance(this.AUTO_ROTATE_TOLERANCE);
        this.m_angleController.enableContinuousInput(-180, 180);

        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_angleController.reset(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        this.m_angleController.setGoal(this.m_angleSetpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // inputModulus will wrap the value to between -180 and 180. This combine with
        // using enableContinuousInput on the PID Controller
        // means that our robot will always take the shortest path to the angle.
        // copied this from windham
        double rotationVal = this.m_angleController.calculate(
                (MathUtil.inputModulus(this.m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), -180, 180)),
                this.m_angleController.getSetpoint());

        this.m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        rotationVal,
                        m_drivetrainSubsystem.getGyroscopeRotation()));
        System.out.println("Setpoint: " + this.m_angleController.getGoal().position + "power " + rotationVal);
        System.out.println("Current angle: "
                + (MathUtil.inputModulus(this.m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), -180, 180)));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // when we finish set the rotation to 0 but keep driving
        this.m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        0,
                        m_drivetrainSubsystem.getGyroscopeRotation()));

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double error = Math
                .abs(m_angleController.getGoal().position - m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        System.out.println("setpoint: " + m_angleSetpoint);
        return error < AUTO_ROTATE_TOLERANCE;
    }
}
