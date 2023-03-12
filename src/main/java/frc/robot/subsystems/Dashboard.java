// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.GrpAutoHighCubeAndBalance;
import frc.robot.subsystems.Arm.ArmPosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
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
	private ArmPosition _armPosition;
	



  	public Dashboard(Arm armSubsystem, Drive driveSubsystem) {
	  	ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
	  	_armSubsystem = armSubsystem;
	  	_driveSubsystem = driveSubsystem;

	  	_autonomousChooser = new SendableChooser<Command>();

	  	dashboard.add("Autonomous Selector", _autonomousChooser)
	  	.withPosition(5, 4)
	  	.withSize(5, 1)
	  	.withWidget(BuiltInWidgets.kComboBoxChooser);

		dashboard.add("Gyro", new AHRS())
	  	.withPosition(8, 0)
	  	.withSize(2, 4)
	  	.withWidget(BuiltInWidgets.kGyro);

		_hasGamepiece = false;
		_armPosition = ArmPosition.Stored;
		ShuffleboardLayout arm = dashboard.getLayout("Arm Subsystem", BuiltInLayouts.kList)
		  .withPosition(5,1)
		  .withSize(3, 3)
		  .withProperties(Map.of("Label position", "BOTTOM"));

		// _hasGamepieceEntry = arm.add("Has Gamepiece", _hasGamepiece).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "Lime", "Color when false", "Red")).getEntry();
		// _armPositionEntry = arm.add("Arm Position", _armPosition.toString()).withWidget(BuiltInWidgets.kTextView).getEntry();

  	}

  	@Override
  	public void periodic() {
		// This method will be called once per scheduler run
		// _hasGamepieceEntry.setBoolean(_hasGamepiece);
		// _armPositionEntry.setString(_armPosition);
  	}

  	public void addAuto(String name, Command command) {
	  	_autonomousChooser.addOption(name, command);
  	}

  	public Command getSelectedAutonomousCommand() {
		//return _autonomousChooser.getSelected();
		//return new GrpAutoHighCubeAndBalance(_driveSubsystem, _armSubsystem);
        return Autonomous.getPreloadedPathCommand("TEST_Short_Distance", 3, 1.5, _driveSubsystem, _driveSubsystem::drive);
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
