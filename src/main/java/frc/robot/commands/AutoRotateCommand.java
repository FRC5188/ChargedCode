package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;

public class AutoRotateCommand extends CommandBase {
    private final Drive m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final double m_rotationSetpoint;

    public AutoRotateCommand(Drive drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier, 
                               double rotationSetpoint) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSetpoint = rotationSetpoint;

        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize(){
        //this.m_drivetrainSubsystem.rotatePIDInit(m_rotationSetpoint);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        // double rotationPower = this.m_drivetrainSubsystem.rotatePIDExec();
        // m_drivetrainSubsystem.drive(
        //         ChassisSpeeds.fromFieldRelativeSpeeds(
        //                 m_translationXSupplier.getAsDouble(),
        //                 m_translationYSupplier.getAsDouble(),
        //                 rotationPower,
        //                 m_drivetrainSubsystem.getGyroscopeRotation()
        //         )
        // );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            0,
            m_drivetrainSubsystem.getGyroscopeRotation()
    ));
    }

    @Override
    public boolean isFinished(){
        //return this.m_drivetrainSubsystem.rotatePIDAtSetpoint();
        return true;
    }
}