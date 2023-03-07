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
import frc.robot.autonomous.Autonomous;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Arm.ArmMode;
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
    private final Drive _driveSubsystem = new Drive();
    private final Arm _armSubsystem = new Arm();
    private final Dashboard _dashboardSubsystem = new Dashboard(_armSubsystem, _driveSubsystem);


    private final XboxController _driverController = new XboxController(0);
    private final JoystickButton _driverButtonRB = new JoystickButton(_driverController, Constants.ButtonMappings.RIGHT_BUMPER);
    private final JoystickButton _driverButtonX = new JoystickButton(_driverController, Constants.ButtonMappings.X_BUTTON);
    private final JoystickButton _driverButtonA = new JoystickButton(_driverController, Constants.ButtonMappings.A_BUTTON);

    
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

    public final Joystick _sliderJoystick = new Joystick(2);
    private JoystickButton _opButtonThirteen = new JoystickButton(_operatorController, 13);

    public final Joystick _toggleSwitch = new Joystick(3);

    // Constant Arm Multiplier In To Reduce Arm Speed
    private static final double ARM_MULTIPLIER = 0.3;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        _sliderJoystick.getRawAxis(0);

        // Driver Configuration
        _driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                                               _driveSubsystem,
                                               () -> (-modifyAxis(_driverController.getLeftY() )* Drive.MAX_VELOCITY_METERS_PER_SECOND * _driveSubsystem.getSpeedMultiplier()),
                                               () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND * _driveSubsystem.getSpeedMultiplier()),
                                               () -> (-modifyAxis(_driverController.getRightX()) * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * _driveSubsystem.getSpeedMultiplier())));
        

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
        _driverButtonRB.onFalse(new CmdDriveChangeSpeedMult(_driveSubsystem, 0.4));
        _driverButtonRB.onTrue(new CmdDriveChangeSpeedMult(_driveSubsystem, 1.0)); 

        // Reset Gyro
       // _driverButtonA.whileTrue(new CmdDriveResetGyro(_driveSubsystem));

        // -- Operator Controls --

        // Toggle between cone and cube mode by holding down trigger
        _opButtonOne.onTrue(new CmdArmSetMode(_armSubsystem, ArmMode.Cone));
        System.out.println("Cone mode");
        _opButtonOne.onFalse(new CmdArmSetMode(_armSubsystem, ArmMode.Cube));
        System.out.println("Cube mode");

        // Go to stow
        _opButtonThree.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.Stored));
        System.out.println("Going to stored");

        // Move arm to position manual
        _opButtonFour.whileTrue(new GrpMoveArmToPositionManual(_armSubsystem, 
        () -> _sliderJoystick.getRawAxis(0), 
        () -> _sliderJoystick.getRawAxis(0)));
        
        // Run intake (shouldn't need this, but just in case)
        _opButtonFive.onTrue(new CmdArmRunIntake(_armSubsystem, -0.4));

        // Spit out game piece
        _opButtonSix.whileTrue(new CmdArmRunIntake(_armSubsystem, 0.4));
        System.out.println("Spitting");


        // Go to high position. Will change based on if you are in cone or cube mode
        _opButtonSeven.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.High));
        System.out.println("Going to high");


        // Go to mid position. Will change based on if you are in cone or cube mode
        _opButtonNine.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.Middle));
        System.out.println("Going to middle");


        // Go to loading station pickup
        _opButtonTen.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.LoadStationPickUp));
        System.out.println("Going to loading station");


        // Go to low position
        _opButtonEleven.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.LowScore));
        System.out.println("Going to low score");


        // Go to ground pickup
        _opButtonTwelve.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.GroundPickUp));

        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return _driveSubsystem.getAutonomousCommand();
    }

    public Command getInitialArmPosCommand() {
        return new CmdArmUpdateGoal(_armSubsystem, ArmPosition.Stored);
    }

    public Command getPIDCommand() {
        return new CmdArmDefault(_armSubsystem, null);
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