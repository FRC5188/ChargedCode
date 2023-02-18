// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.Tracer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.CmdArmRunIntake;
import frc.robot.commands.CmdArmManual;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive _driveSubsystem;
    private final Arm _armSubsystem;
    // Controllers 
    private final XboxController _driverController;
    private final Joystick _operatorController;
    // Driver Buttons

    // Codriver Buttons
    private Trigger _operatorButtonOne;
    private Trigger _operatorButtonTwo;

    // Constant Arm Multiplier In To Reduce Arm Speed
    private static final double ARM_MULTIPLIER = 0.3;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        _driveSubsystem = new Drive();
        _armSubsystem = new Arm();
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        _driverController = new XboxController(0);
        _operatorController = new Joystick(1);
        // Driver Configuration
        _driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                                               _driveSubsystem,
                                               () -> (-modifyAxis(_driverController.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                                               () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                                               () -> (-modifyAxis(_driverController.getRightX()) * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));
        // Codriver Configuration
        // If the throttle is in one state then the Z-axis will control one motor, and when its moved then it controlled the other. 
        _armSubsystem.setDefaultCommand(new CmdArmManual(
            _armSubsystem, 
            () -> (getShoulderSpeed()), 
            () -> (getArmSpeed())
        ));
        _operatorButtonOne = new JoystickButton(_operatorController, 1);
        _operatorButtonTwo = new JoystickButton(_operatorController, 2);
        _operatorButtonOne.whileTrue(new CmdArmRunIntake(_armSubsystem, () -> (0.5)));
    }

    private double getShoulderSpeed(){
        return ARM_MULTIPLIER*((_operatorController.getThrottle() > 0) ? (0) : (_operatorController.getZ()));
    }

    private double getArmSpeed(){
        return ARM_MULTIPLIER*((_operatorController.getThrottle() > 0) ? (_operatorController.getZ()) : (0));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        // new Button(m_controller::getBackButton)
        // // No requirements because we don't need to interrupt anything
        // .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

        //_operatorAButton.onTrue(new CmdArmRunIntake(_armSubsystem));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new InstantCommand();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}