// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class CmdArmRunIntakeByPosition extends CommandBase {
    private Arm _armSubsystem;
    private ArmPosition _position;
    private double _intakeSpeed;

    public CmdArmRunIntakeByPosition(Arm armSubsystem, ArmPosition position, double intakeSpeed) {
        this._armSubsystem = armSubsystem;
        _position = position;
        this._intakeSpeed = intakeSpeed;

        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {
        // We want the intake to start running only in certain positions
        // If it tries to run in a position we don't want it to run in, 
        // cancel the command
        if (_position != ArmPosition.GroundPickUp || _position != ArmPosition.LoadStationPickUp) {
          this.cancel();
        }
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
