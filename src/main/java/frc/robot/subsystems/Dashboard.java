// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  	Vision _visionSubsystem;

  	private SendableChooser<Command> _autonomousChooser;



  	public Dashboard(Arm armSubsystem, Drive driveSubsystem, Vision visionSubsystem) {
	  	ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
	  	_armSubsystem = armSubsystem;
	  	_driveSubsystem = driveSubsystem;
	  	_visionSubsystem = visionSubsystem;

	  	_autonomousChooser = new SendableChooser<Command>();

	  	dashboard.add("Autonomous Selector", _autonomousChooser)
	  	.withPosition(5, 4)
	  	.withSize(5, 1)
	  	.withWidget(BuiltInWidgets.kComboBoxChooser);

		dashboard.add("Gyro", new AHRS())
	  	.withPosition(8, 0)
	  	.withSize(2, 4)
	  	.withWidget(BuiltInWidgets.kGyro);



  	}

  	@Override
  	public void periodic() {
		// This method will be called once per scheduler run
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
}
