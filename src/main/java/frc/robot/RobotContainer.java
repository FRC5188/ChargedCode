// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.Autonomous;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LEDs;
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
    private final LEDs _leds = new LEDs();
 
    private final XboxController _driverController = new XboxController(0);
    private final JoystickButton _driverButtonRB = new JoystickButton(_driverController, Constants.ButtonMappings.RIGHT_BUMPER);
    private final JoystickButton _driverButtonY = new JoystickButton(_driverController, Constants.ButtonMappings.Y_BUTTON);
    private final JoystickButton _driverButtonA = new JoystickButton(_driverController, Constants.ButtonMappings.A_BUTTON);

    //Top row of buttons
    private final Joystick _operatorController1 = new Joystick(1);
    //Bottomrow of buttons
    private final Joystick _operatorController2 = new Joystick(2);
    
    private JoystickButton _opButtonOne = new JoystickButton(_operatorController1, 1);
    private JoystickButton _opButtonTwo = new JoystickButton(_operatorController1, 2);
    private JoystickButton _opButtonThree = new JoystickButton(_operatorController1, 3);
    private JoystickButton _opButtonFour = new JoystickButton(_operatorController1, 4);
    private JoystickButton _opButtonFive = new JoystickButton(_operatorController1, 5);
    private JoystickButton _opButtonSix = new JoystickButton(_operatorController1, 6);
    private JoystickButton _opButtonSeven = new JoystickButton(_operatorController1, 7);
    private JoystickButton _opButtonEight = new JoystickButton(_operatorController1, 8);
    private JoystickButton _opButtonNine = new JoystickButton(_operatorController1, 9);
    private JoystickButton _opButtonTen = new JoystickButton(_operatorController1, 10);
    private JoystickButton _opButtonEleven = new JoystickButton(_operatorController1, 11);
    private JoystickButton _opButtonTwelve = new JoystickButton(_operatorController1, 12);

    // private JoystickButton _opToggle = new JoystickButton(_operatorController2, 10);
    private JoystickButton _op2ButtonOne = new JoystickButton(_operatorController2, 1);
    private JoystickButton _op2ButtonTwo = new JoystickButton(_operatorController2, 2);

    public final Joystick _sliderJoystick = new Joystick(2);
    private JoystickButton _opButtonThirteen = new JoystickButton(_operatorController1, 13);

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

        // _driverButtonA.onTrue(new AutoRotateCommand(
        //     _driveSubsystem,
        //     () -> (-modifyAxis(_driverController.getLeftY() )* Drive.MAX_VELOCITY_METERS_PER_SECOND * _driveSubsystem.getSpeedMultiplier()),
        //     () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND * _driveSubsystem.getSpeedMultiplier()),
        //     0));
        //_driverButtonA.onTrue(new CmdDriveResetGyro(_driveSubsystem));
        //_driverButtonA.onTrue(new CmdArmSpit(_armSubsystem, 0.4));
        _driverButtonY.onTrue(new GrpMoveArmToScore(_armSubsystem));

        // Reset Gyro
       // _driverButtonA.whileTrue(new CmdDriveResetGyro(_driveSubsystem));

        // -- Operator Controls --

        // Run Intake
        _opButtonTwo.onTrue(new CmdArmRunIntake(_armSubsystem, -0.4));

        // Spit game piece
        _opButtonThree.onTrue(new CmdArmSpit(_armSubsystem, 0.4));

        // Loading Station
        _opButtonFour.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.LoadStationPickUp));

        _opButtonFive.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.Stored));

        _opButtonSix.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.GroundPickUp));

        _opButtonSeven.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.High));

        _opButtonEight.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.Middle));

        _opButtonNine.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.LowScore));

        _opButtonTen.onTrue(new CmdArmSetMode(_armSubsystem, ArmMode.Cone));
        _opButtonTen.onFalse(new CmdArmSetMode(_armSubsystem, ArmMode.Cube));

        _opButtonOne.whileTrue(new CmdEnablePID(_armSubsystem));
        _opButtonOne.whileFalse(new CmdDisablePID(_armSubsystem));
        
        // Move arm to position manual
        double elbowUpAmount = 5.0;
        double elbowDownAmount = -5.0;
        double shoulderUpAmount = 3.0;
        double shoulderDownAmount = -3.0;

        // Elbow manual buttons
        _op2ButtonOne.onTrue(new CmdMoveElbowManual(_armSubsystem, elbowDownAmount));
        _op2ButtonTwo.onTrue(new CmdMoveElbowManual(_armSubsystem, elbowUpAmount));

        // Shoulder manual buttons
        _op2ButtonOne.onTrue(new CmdMoveShoulderManual(_armSubsystem, shoulderDownAmount));
        _op2ButtonTwo.onTrue(new CmdMoveShoulderManual(_armSubsystem, shoulderUpAmount));
        // _opButtonThirteen.whileTrue(new GrpMoveArmToPositionManual(_armSubsystem, 
        // () -> _sliderJoystick.getRawAxis(0), 
        // () -> _sliderJoystick.getRawAxis(0)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("HighCubeArm", new GrpMoveArmToPosition(_armSubsystem, ArmPosition.HighCube));
        eventMap.put("Spit", new CmdArmSpit(_armSubsystem, 0.4));
        eventMap.put("StoreArm", new GrpMoveArmToPosition(_armSubsystem, ArmPosition.Stored));
        //return Autonomous.generateFullAuto("DriveBackwardsToPlatform", eventMap, 3, 0.5, _driveSubsystem);

        //return Autonomous.generateFullAuto("HighScoreAndMobility", eventMap, 3, 0.5, _driveSubsystem);

        return _dashboardSubsystem.getSelectedAutonomousCommand();
    }

    public Command getInitialArmPosCommand() {
        return new CmdArmUpdateGoal(_armSubsystem, ArmPosition.Stored);
    }

    public Command getPIDCommand() {
        return new CmdArmDefault(_armSubsystem);
    }

    public Command updateLEDs() {
        return new CmdLEDDefault(_leds, _armSubsystem);
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