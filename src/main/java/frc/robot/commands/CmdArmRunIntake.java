// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdArmRunIntake extends CommandBase {
    private Arm _armSubsystem;
    private DoubleSupplier _intakeSpeed;

    // eventually we probably wont want to just pass in a speed. This is for testing
    // the arm on 2/4/23
    public CmdArmRunIntake(Arm armSubsystem, DoubleSupplier intakeSpeed) {
        this._armSubsystem = armSubsystem;
        this._intakeSpeed = intakeSpeed;
        // Add the required subsytems.
        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Set the values give from the joystick to the shoulder and elbow.
        this._armSubsystem.setIntakeMotorSpeed(this._intakeSpeed.getAsDouble());
        //System.out.println(this._armSubsystem.getChangeInIntakeMotorCurrent());
    }



    @Override
    public boolean isFinished() {
        return false;
    }
}
