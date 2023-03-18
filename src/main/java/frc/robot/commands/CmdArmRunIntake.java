// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.IntakeMode;

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

        //If the gampiece is a cone, close the intake and run the motors.
         if (_armSubsystem.getArmMode() == ArmMode.Cone) {
            _armSubsystem.setIntakeMode(IntakeMode.CLOSED);
            this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
        }
        
        //If the gampiece is a cube, open the intake and run the motors.
        if (_armSubsystem.getArmMode() == ArmMode.Cube) {
            _armSubsystem.setIntakeMode(IntakeMode.OPEN);            
            this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _armSubsystem.setIntakeMotorSpeed(0);
        _armSubsystem.setIntakeMode(IntakeMode.CLOSED);
    }

    @Override
    public boolean isFinished() {
        return _armSubsystem.intakeHasPiece();
    }
}
