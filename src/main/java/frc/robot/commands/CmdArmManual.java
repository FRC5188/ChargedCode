// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// mitchell :)
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class CmdArmManual extends CommandBase {
    private Arm _armSubsystem;
    private DoubleSupplier _shoulderSupplier;
    private DoubleSupplier _elbowSupplier;

    public CmdArmManual(Arm armSubsystem, DoubleSupplier shoulderSupplier, DoubleSupplier elbowSupplier) {
        this._armSubsystem = armSubsystem;
        this._shoulderSupplier = shoulderSupplier;
        this._elbowSupplier = elbowSupplier;

        // Add the required subsytems.
        this.addRequirements(_armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Set the values give from the joystick to the shoulder and elbow.
        this._armSubsystem.setShoulderMotorSpeed(_shoulderSupplier.getAsDouble());
        this._armSubsystem.setElbowMotorSpeed(_elbowSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
