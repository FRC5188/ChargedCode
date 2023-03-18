// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.dashboard;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.autonomous.Autonomous;
import frc.robot.drive.Drive;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Dashboard extends SubsystemBase {

  	/** Creates a new Dashboard. */
  	Arm _armSubsystem;
 	Drive _driveSubsystem;

  	private SendableChooser<Command> _autonomousChooser;
	private NetworkTableEntry _hasGamepieceEntry;
	private NetworkTableEntry _armPositionEntry;
	private boolean _hasGamepiece;
	private boolean _elbowAtSetpoint;
	private boolean _shoulderAtSetpoint;
	private ArmPosition _armPosition;

  	public Dashboard(Arm armSubsystem, Drive driveSubsystem) {
	  	ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
	  	_armSubsystem = armSubsystem;
	  	_driveSubsystem = driveSubsystem;
		_hasGamepiece = _armSubsystem.checkGamepiece();
		_elbowAtSetpoint = _armSubsystem.elbowAtSetpoint();
		_shoulderAtSetpoint = _armSubsystem.shoulderAtSetpoint();
		
	  	_autonomousChooser = new SendableChooser<Command>();

	  	dashboard.add("Autonomous Selector", _autonomousChooser)
	  	.withPosition(23, 0)
	  	.withSize(8, 2)
	  	.withWidget(BuiltInWidgets.kComboBoxChooser);

		/*dashboard.add("Camera Stream", new UsbCamera(getName(), 0))
		.withPosition(0,0)
		.withSize(23,14)
		.withWidget(BuiltInWidgets.kCameraStream);*/

		dashboard.add("Gyro", _driveSubsystem.getGyroInstance())
	  	.withPosition(23, 8)
	  	.withSize(5, 5)
	  	.withWidget(BuiltInWidgets.kGyro);

		dashboard.add("Has Gamepiece", _hasGamepiece)
		.withPosition(23, 2)
		.withSize(3, 3)
		.withWidget(BuiltInWidgets.kBooleanBox)
		.withProperties(Map.of("Color when true", "Lime", "Color wben false", "Red")).getEntry();

		dashboard.add("Elbow @ Setpoint", _elbowAtSetpoint)
		.withPosition(26, 2)
		.withSize(3, 3)
		.withWidget(BuiltInWidgets.kBooleanBox)
		.withProperties(Map.of("Color when true", "Lime", "Color wben false", "Red")).getEntry();


		dashboard.add("Shoulder @ Setpoint", _shoulderAtSetpoint)
		.withPosition(23, 5)
		.withSize(6, 3)
		.withWidget(BuiltInWidgets.kBooleanBox)
		.withProperties(Map.of("Color when true", "Lime", "Color wben false", "Red")).getEntry();


		_armPosition = ArmPosition.Stored;

		/*ShuffleboardLayout arm = dashboard.getLayout("Arm Subsystem", BuiltInLayouts.kList)
		  .withPosition(5,1)
		  .withSize(3, 3)
		  .withProperties(Map.of("Label position", "BOTTOM"));*/


		// _hasGamepieceEntry = arm.add("Has Gamepiece", _hasGamepiece).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "Lime", "Color when false", "Red")).getEntry();
		// _armPositionEntry = arm.add("Arm Position", _armPosition.toString()).withWidget(BuiltInWidgets.kTextView).getEntry();

  	}

  	@Override
  	public void periodic() {
		_hasGamepiece = _armSubsystem.checkGamepiece();
		_elbowAtSetpoint = _armSubsystem.elbowAtSetpoint();
		_shoulderAtSetpoint = _armSubsystem.shoulderAtSetpoint();

		// This method will be called once per scheduler run
		// _hasGamepieceEntry.setBoolean(_hasGamepiece);
		// _armPositionEntry.setString(_armPosition);
  	}

  	public void addAuto(String name, Command command) {
        _autonomousChooser.addOption(name, command);
    }

    public Command getSelectedAutonomousCommand() {
        return _autonomousChooser.getSelected();
    }

	public void setDefaultAuto(String name, Command command) {
		_autonomousChooser.setDefaultOption(name, command);
	}

	public void setHasGamepiece(boolean state) {
		_hasGamepiece = state;
	}

	public void setArmPosition(ArmPosition state) {
		_armPosition = state;
	}

	// For the future: a battery voltage tracker w/ DriverStation.getInstance().getBatteryVoltage()
}
