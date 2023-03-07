// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdArmRunIntake extends CommandBase {
    private Arm _armSubsystem;
    private double _intakeSpeed;

    public CmdArmRunIntake(Arm armSubsystem, double intakeSpeed) {
        this._armSubsystem = armSubsystem;
        this._intakeSpeed = intakeSpeed;

        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        _armSubsystem.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return _armSubsystem.intakeHasPiece();
    }
}
