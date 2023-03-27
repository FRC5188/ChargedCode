// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commands;

import frc.robot.arm.Arm;
import frc.robot.arm.Arm.WristPosition;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdArmSetWristPositionManual extends CommandBase {
  /** Creates a new CmdArmSetWristPositionManual. */
    private Arm _armSubsystem;
    private WristPosition _pos;

    public CmdArmSetWristPositionManual(Arm armSubsystem, WristPosition pos) {
        this._armSubsystem = armSubsystem;
        _pos = pos;

        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {
        _armSubsystem.setWristPosition(_pos);
        System.out.println("set wrist position to " + _pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
