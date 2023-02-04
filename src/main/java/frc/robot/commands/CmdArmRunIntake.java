// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdArmRunIntake extends CommandBase {
    private Arm _armSubsystem;

    public CmdArmRunIntake(Arm armSubsystem) {
        this._armSubsystem = armSubsystem;
        // Add the required subsytems.
        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Set the values give from the joystick to the shoulder and elbow.
        this._armSubsystem.setIntakeMotorSpeed(0.5);
        System.out.println(this._armSubsystem.getChangeInIntakeMotorCurrent());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
