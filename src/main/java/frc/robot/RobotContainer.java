// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.CmdArmRunIntake;
import frc.robot.commands.CmdArmUpdateGoal;
import frc.robot.commands.CmdArmWristPosition;
import frc.robot.commands.CmdDriveChangeSpeedMult;
import frc.robot.commands.CmdArmDefault;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm.ArmPosition;

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
  // The robot's subsystems and commands are defined here...
    private final Vision _visionSubsystem = new Vision();
    private final Drive _driveSubsystem = new Drive(_visionSubsystem);
    private final Arm _armSubsystem = new Arm();

    private final XboxController _driverController = new XboxController(0);
    private final JoystickButton _driverButtonX = new JoystickButton(_driverController, Constants.ButtonMappings.X_BUTTON);
    
    private final Joystick _operatorController = new Joystick(1);
    
    private JoystickButton _opButtonOne = new JoystickButton(_operatorController, 1);
    private JoystickButton _opButtonTwo = new JoystickButton(_operatorController, 2);
    private JoystickButton _opButtonThree = new JoystickButton(_operatorController, 3);
    private JoystickButton _opButtonFour = new JoystickButton(_operatorController, 4);
    private JoystickButton _opButtonFive = new JoystickButton(_operatorController, 5);
    private JoystickButton _opButtonSix = new JoystickButton(_operatorController, 6);
    private JoystickButton _opButtonSeven = new JoystickButton(_operatorController, 7);
    private JoystickButton _opButtonEight = new JoystickButton(_operatorController, 8);
    private JoystickButton _opButtonNine = new JoystickButton(_operatorController, 9);
    private JoystickButton _opButtonTen = new JoystickButton(_operatorController, 10);
    private JoystickButton _opButtonEleven = new JoystickButton(_operatorController, 11);
    private JoystickButton _opButtonTwelve = new JoystickButton(_operatorController, 12);

    // Constant Arm Multiplier In To Reduce Arm Speed
    private static final double ARM_MULTIPLIER = 0.3;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Driver Configuration
        _driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                                               _driveSubsystem,
                                               () -> (-modifyAxis(_driverController.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND * _driveSubsystem.getSpeedMultiplier()),
                                               () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND * _driveSubsystem.getSpeedMultiplier()),
                                               () -> (-modifyAxis(_driverController.getRightX()) * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * _driveSubsystem.getSpeedMultiplier())));
        
        _armSubsystem.setDefaultCommand(new CmdArmDefault(_armSubsystem));

        configureButtonBindings();
    }

    // private double getShoulderSpeed(){
    //     return ARM_MULTIPLIER*((_operatorController.getLeftY()> 0) ? (0) : (_operatorController.getZ()));
    // }

    // private double getArmSpeed(){
    //     return ARM_MULTIPLIER*((_operatorController.getThrottle() > 0) ? (_operatorController.getZ()) : (0));
    // }

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

        // Driver Reduce Speed 40%
        _driverButtonX.whileFalse(new CmdDriveChangeSpeedMult(_driveSubsystem, 0.4));
        _driverButtonX.whileTrue(new CmdDriveChangeSpeedMult(_driveSubsystem, 1.0)); 

        // Intake On/Off
        _opButtonTwo.whileTrue(new CmdArmRunIntake(_armSubsystem, 0.4));

        // Set Wrist Position
        _opButtonOne.whileTrue(new CmdArmWristPosition(_armSubsystem));

        // Flip Around Intake
        _opButtonEight.whileTrue(new CmdArmRunIntake(_armSubsystem, -0.4));

        //_opButtonSeven.whileTrue(new CmdArmUpdateGoal(_armSubsystem, ArmPosition.LowScore));
        _opButtonNine.whileTrue(new CmdArmUpdateGoal(_armSubsystem, ArmPosition.Stored));
        _opButtonTen.whileTrue(new CmdArmUpdateGoal(_armSubsystem, ArmPosition.LoadStationPickUp));
        _opButtonThree.whileTrue(new CmdArmUpdateGoal(_armSubsystem, ArmPosition.MiddleCube));
        _opButtonFour.whileTrue(new CmdArmUpdateGoal(_armSubsystem, ArmPosition.MiddleCone));
        _opButtonFive.whileTrue(new CmdArmUpdateGoal(_armSubsystem, ArmPosition.HighCube));
        _opButtonSix.whileTrue(new CmdArmUpdateGoal(_armSubsystem, ArmPosition.HighCone));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return _driveSubsystem.followTrajectoryCommand(
      PathPlanner.loadPath("TEST_Straight_Line", 3, 4), 
          true);
    }

    public Command getInitialArmPosCommand() {
        return new CmdArmUpdateGoal(_armSubsystem, ArmPosition.Stored);
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