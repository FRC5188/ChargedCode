// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonMappings;
import frc.robot.LEDs.LEDs;
import frc.robot.LEDs.commands.CmdLEDAnimation;
import frc.robot.LEDs.commands.CmdLEDDefault;
import frc.robot.LEDs.commands.CmdLEDPieceCollected;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commandGroups.GrpAutoEngardeScoreStow;
import frc.robot.arm.commandGroups.GrpAutoGoToScore;
import frc.robot.arm.commandGroups.GrpEngardeForScoring;
import frc.robot.arm.commandGroups.GrpEngardeScoreAndStow;
import frc.robot.arm.commandGroups.GrpMoveArmToPosition;
import frc.robot.arm.commandGroups.GrpMoveArmToScore;
import frc.robot.arm.commandGroups.GrpScoreAndStow;
import frc.robot.arm.commands.CmdArmDefault;
import frc.robot.arm.commands.CmdArmRunIntake;
import frc.robot.arm.commands.CmdArmSetMode;
import frc.robot.arm.commands.CmdArmSpit;
import frc.robot.arm.commands.CmdArmUpdateGoal;
import frc.robot.arm.commands.CmdArmUpdateToFinalPosition;
import frc.robot.arm.commands.CmdArmUpdateToLastTarget;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.Autonomous.FIELD_POSITIONS;
import frc.robot.arm.commands.CmdArmDisablePID;
import frc.robot.arm.commands.CmdArmEnablePID;
import frc.robot.arm.commands.CmdArmMoveElbowManual;
import frc.robot.arm.commands.CmdArmMoveShoulderManual;
import frc.robot.dashboard.Dashboard;
import frc.robot.drive.Drive;
import frc.robot.vision.Limelight;
import frc.robot.drive.commands.CmdDriveAutoBalance;
import frc.robot.drive.commands.CmdDriveAutoRotate;
import frc.robot.drive.commands.CmdDriveChangeCoR;
import frc.robot.drive.commands.CmdDriveChangeSpeedMult;
import frc.robot.drive.commands.CmdDriveResetGyro;
import frc.robot.drive.commands.DefaultDriveCommand;

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
        public final Drive _driveSubsystem = new Drive();
        private final Arm _armSubsystem = new Arm();
        private final Dashboard _dashboardSubsystem = new Dashboard(_armSubsystem, _driveSubsystem);
        public final LEDs _leds = new LEDs();

        private final XboxController _driverController = new XboxController(0);

        private final JoystickButton _driverButtonX = new JoystickButton(_driverController,
                        Constants.ButtonMappings.X_BUTTON);
        private final JoystickButton _driverButtonB = new JoystickButton(_driverController,
                        Constants.ButtonMappings.B_BUTTON);

        private final JoystickButton _driverButtonRB = new JoystickButton(_driverController,
                        Constants.ButtonMappings.RIGHT_BUMPER);
        private final JoystickButton _driverButtonY = new JoystickButton(_driverController,
                        Constants.ButtonMappings.Y_BUTTON);
        private final JoystickButton _driverButtonA = new JoystickButton(_driverController,
                        Constants.ButtonMappings.A_BUTTON);
        private final JoystickButton _driverButtonLB = new JoystickButton(_driverController,
                        Constants.ButtonMappings.LEFT_BUMPER);
        private final JoystickButton _driverButtonRightJoyButton = new JoystickButton(_driverController,
                        Constants.ButtonMappings.RIGHT_JOY_BUTTON);

        // Top row of buttons
        private final Joystick _operatorController1 = new Joystick(1);
        // Bottom row of buttons
        private final Joystick _operatorController2 = new Joystick(2);
        // manual sliders

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

        // private JoystickButton _opToggle = new JoystickButton(_operatorController2,
        // 10);
        private JoystickButton _op2ButtonOne = new JoystickButton(_operatorController2, 1);
        private JoystickButton _op2ButtonTwo = new JoystickButton(_operatorController2, 2);
        private JoystickButton _op2ButtonThree = new JoystickButton(_operatorController2, 3);
        private JoystickButton _op2ButtonFour = new JoystickButton(_operatorController2, 4);
        private JoystickButton _op2ButtonFive = new JoystickButton(_operatorController2, 5);
        private JoystickButton _op2ButtonSix = new JoystickButton(_operatorController2, 6);
        private JoystickButton _op2ButtonNine = new JoystickButton(_operatorController2, 9);


        public final Joystick _sliderJoystick = new Joystick(2);
        private JoystickButton _opButtonThirteen = new JoystickButton(_operatorController1, 13);

        public final Joystick _toggleSwitch = new Joystick(3);

        // Constant Arm Multiplier In To Reduce Arm Speed
        private static final double ARM_MULTIPLIER = 0.3;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                Trigger hasGamePieceTrigger = new Trigger(() -> (_armSubsystem.checkGamepiece()));
                hasGamePieceTrigger.onTrue(new CmdLEDPieceCollected(_leds));

                //_leds.setDefaultCommand(new CmdLEDDefault(_leds, _armSubsystem));

                // Driver Configuration
                _driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                                _driveSubsystem,
                                () -> (-modifyAxis(_driverController.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                () -> (-modifyAxis(_driverController.getRightX())
                                                * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier())));

                HashMap<String, Command> eventMap = generateEventMap();

                _dashboardSubsystem.setDefaultAuto("High Cube Mobility",
                                Autonomous.generateFullAuto("HighCubeMobility", eventMap, 3, 0.75,
                                                _driveSubsystem));

                _dashboardSubsystem.addAuto("High Cube Mobility Auto Balance",
                                Autonomous.generateFullAuto("HighCubeMobilityBalance", eventMap, 3, 0.75,
                                                _driveSubsystem));

                _dashboardSubsystem.addAuto("2.5 Piece Cubes No Bump",
                                Autonomous.generateFullAuto("2.5PieceWithPassiveNoBump", eventMap, 3.5, 1.5,
                                                _driveSubsystem));

                _dashboardSubsystem.addAuto("2.5 Piece Cubes With Bump",
                                Autonomous.generateFullAuto("2.5PieceWithPassiveBump", eventMap, 3.5, 1.5,
                                                _driveSubsystem));

                _dashboardSubsystem.addAuto("1.5 Piece Cone",
                                Autonomous.generateFullAuto("1.5PieceAuto", eventMap, 3.5, 1.25,
                                                _driveSubsystem));

                _dashboardSubsystem.addAuto("Move a Meter",
                                Autonomous.generateFullAuto("TEST_Short_Distance", eventMap, 4, 1, _driveSubsystem));

                configureButtonBindings();
        }

        // private double getShoulderSpeed(){
        // return ARM_MULTIPLIER*((_operatorController.getLeftY()> 0) ? (0) :
        // (_operatorController.getZ()));
        // }
        // private double getArmSpeed(){
        // return ARM_MULTIPLIER*((_operatorController.getThrottle() > 0) ?
        // (_operatorController.getZ()) : (0));
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

                // Driver Reduce Speed 40%
                _driverButtonRB.onFalse(new CmdDriveChangeSpeedMult(_driveSubsystem, 0.4));
                _driverButtonRB.onTrue(new CmdDriveChangeSpeedMult(_driveSubsystem, 1.0));

                // Change CoR for orbiting around a cone or robot

                _driverButtonRightJoyButton
                                .whileTrue(new CmdDriveChangeCoR(_driveSubsystem, new Translation2d(1.07, 0)));
                _driverButtonRightJoyButton.whileFalse(new CmdDriveChangeCoR(_driveSubsystem, new Translation2d(0, 0)));

                _driverButtonA.whileTrue(new CmdDriveAutoRotate(
                                _driveSubsystem,
                                () -> (-modifyAxis(_driverController.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                180));

                // Counter clockwise is positive angle
                _driverButtonB.whileTrue(new CmdDriveAutoRotate(
                                _driveSubsystem,
                                () -> (-modifyAxis(_driverController.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                -90));

                _driverButtonX.whileTrue(new CmdDriveAutoRotate(
                                _driveSubsystem,
                                () -> (-modifyAxis(_driverController.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                90));

                _driverButtonY.whileTrue(new CmdDriveAutoRotate(
                                _driveSubsystem,
                                () -> (-modifyAxis(_driverController.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                () -> (-modifyAxis(_driverController.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND
                                                * _driveSubsystem.getSpeedMultiplier()),
                                0));

                // TODO: change selector to boolean that returns true if our target and current
                // pos is en garde and false otherwise
                _driverButtonLB.onTrue(new SelectCommand(
                                // Maps selector values to commands
                                Map.ofEntries(
                                                Map.entry(false, new CmdArmSpit(_armSubsystem, -0.6)),
                                                Map.entry(true,
                                                                new CmdArmUpdateToFinalPosition(_armSubsystem)
                                                                                .unless(() -> !_armSubsystem
                                                                                                .canChangeSetpoint()))),
                                () -> _armSubsystem.getTargetArmPosition() == ArmPosition.EnGarde));

                // -- Operator Controls --
                _opButtonOne.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.EnGarde)
                                .unless(() -> !_armSubsystem.canChangeSetpoint()));

                // Run Intake
                _opButtonTwo.onTrue(new CmdArmRunIntake(_armSubsystem, 1));

                // Spit game piece
                _opButtonThree.onTrue(new CmdArmSpit(_armSubsystem, -0.6));

                // Loading Station
                _opButtonFour.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.LoadStationPickUp)
                                .unless(() -> !_armSubsystem.canChangeSetpoint()));

                _opButtonFive.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.Stored)
                                .unless(() -> !_armSubsystem.canChangeSetpoint()));

                _opButtonSix.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.GroundPickUp)
                                .unless(() -> (!_armSubsystem.canChangeSetpoint() || (_armSubsystem
                                                .getCurrentArmPosition() != ArmPosition.Stored
                                                && _armSubsystem.getCurrentArmPosition() != ArmPosition.EnGarde))));

                _opButtonSeven.onTrue(new GrpEngardeForScoring(_armSubsystem, ArmPosition.High)
                                .unless(() -> !_armSubsystem.canChangeSetpoint()));

                _opButtonEight.onTrue(new GrpEngardeForScoring(_armSubsystem, ArmPosition.Middle)
                                .unless(() -> !_armSubsystem.canChangeSetpoint()));

                _opButtonNine.onTrue(new GrpMoveArmToPosition(_armSubsystem, ArmPosition.LowScore)
                                .unless(() -> !_armSubsystem.canChangeSetpoint()));

                _opButtonTen.onTrue(new CmdArmSetMode(_armSubsystem, ArmMode.Cone));
                _opButtonTen.onFalse(new CmdArmSetMode(_armSubsystem, ArmMode.Cube));

                _opButtonOne.whileTrue(new CmdArmEnablePID(_armSubsystem));
                _opButtonOne.whileFalse(new CmdArmDisablePID(_armSubsystem));

                // Move arm to position manual
                double elbowUpAmount = 3.0;
                double elbowDownAmount = -3.0;
                double shoulderUpAmount = 3.0;
                double shoulderDownAmount = -3.0;

                // Elbow manual buttons
                _op2ButtonOne.onTrue(new CmdArmMoveElbowManual(_armSubsystem, () -> elbowDownAmount));
                _op2ButtonTwo.onTrue(new CmdArmMoveElbowManual(_armSubsystem, () -> elbowUpAmount));

                // Shoulder manual buttons
                _op2ButtonThree.onTrue(new CmdArmMoveShoulderManual(_armSubsystem, () -> shoulderDownAmount));
                _op2ButtonFour.onTrue(new CmdArmMoveShoulderManual(_armSubsystem, () -> shoulderUpAmount));

                _op2ButtonFive.onTrue(
                                new InstantCommand(() -> _armSubsystem
                                                .setCurrentPosition(_armSubsystem.getTargetArmPosition())));

                _op2ButtonNine.whileTrue(new InstantCommand(() -> _armSubsystem.setCanChangeSetpoint(true)));
                _op2ButtonNine.whileFalse(new InstantCommand(() -> _armSubsystem.setCanChangeSetpoint(false)));

                // this currently infinitely adds to the current setpoint while button is held
                // _opButtonOne.whileTrue(Commands.parallel(
                // new CmdArmMoveElbowManual(
                // _armSubsystem,
                // () -> (_sliderJoystick.getRawAxis(0))),
                // new CmdArmMoveShoulderManual(
                // _armSubsystem,
                // () -> (_sliderJoystick.getRawAxis(1)))).repeatedly());

        }

        private HashMap<String, Command> generateEventMap() {
                HashMap<String, Command> eventMap = new HashMap<>();

                eventMap.put("ScoreCone", new GrpAutoGoToScore(_armSubsystem, ArmPosition.HighCone));
                eventMap.put("ScoreCube", new GrpAutoGoToScore(_armSubsystem, ArmPosition.HighCube));
                eventMap.put("HighCubeArm", new GrpAutoGoToScore(_armSubsystem, ArmPosition.HighCube));
                eventMap.put("HighConeArm", new GrpAutoGoToScore(_armSubsystem, ArmPosition.HighCone));
                eventMap.put("HighConeScoreAndStow", new GrpAutoEngardeScoreStow(_armSubsystem, ArmPosition.HighCone));
                eventMap.put("Spit", new CmdArmSpit(_armSubsystem, 0.6));
                eventMap.put("GroundPickupCube",
                                new SequentialCommandGroup(new CmdArmSetMode(_armSubsystem, ArmMode.Cube),
                                                new GrpMoveArmToPosition(_armSubsystem, ArmPosition.GroundPickUp)));
                eventMap.put("Stow", new GrpMoveArmToPosition(_armSubsystem, ArmPosition.Stored));
                eventMap.put("EnGarde", new GrpMoveArmToPosition(_armSubsystem, ArmPosition.EnGarde));
                eventMap.put("HighCube", new SequentialCommandGroup(new CmdArmSetMode(_armSubsystem, ArmMode.Cube),
                                new GrpMoveArmToPosition(_armSubsystem, ArmPosition.HighCube)));
                eventMap.put("ScoreAndStow", new GrpScoreAndStow(_armSubsystem));
                eventMap.put("EnGardeAndGPCube",
                                new SequentialCommandGroup(new CmdArmSetMode(_armSubsystem, ArmMode.Cube),
                                                new GrpMoveArmToPosition(_armSubsystem, ArmPosition.EnGarde),
                                                new InstantCommand(() -> _armSubsystem
                                                                .setCurrentPosition(
                                                                                _armSubsystem.getTargetArmPosition())),
                                                new GrpMoveArmToPosition(_armSubsystem, ArmPosition.GroundPickUp)));
                eventMap.put("EnGardeAndHighCube",
                                new SequentialCommandGroup(new CmdArmSetMode(_armSubsystem, ArmMode.Cube),
                                                new GrpMoveArmToPosition(_armSubsystem, ArmPosition.EnGarde),
                                                new InstantCommand(() -> _armSubsystem
                                                                .setCurrentPosition(
                                                                                _armSubsystem.getTargetArmPosition())),
                                                new GrpMoveArmToPosition(_armSubsystem, ArmPosition.HighCube)));
                eventMap.put("Balance", new CmdDriveAutoBalance(_driveSubsystem));
                eventMap.put("Score", new CmdArmSpit(_armSubsystem, -0.6));

                return eventMap;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                return _dashboardSubsystem.getSelectedAutonomousCommand();
                // return
                // Autonomous.getMovementCommand(FIELD_POSITIONS.LEFT_SIDE_GRID_CUBE_SECOND_CLOSEST,
                // 3, 4, _driveSubsystem, null)
                // return Autonomous.generateFullAuto("HighScoreAndMobility", eventMap, 3, 0.5,
                // _driveSubsystem);

                // return new CmdDriveAutoBalance(_driveSubsystem);
        }

        public Command getInitialArmPosCommand() {
                return new CmdArmUpdateToLastTarget(_armSubsystem);
        }

        public Command getPIDCommand() {
                return new CmdArmDefault(_armSubsystem);
        }

        public Command updateLEDs() {
                System.out.println("Cmd UpdateLEDs");
                return new CmdLEDDefault(_leds, _armSubsystem);
        }

        public Command setLEDAnim(){
                System.out.println("Cmd UpdateLEDANIM");
                new CmdLEDAnimation(_leds);
                return new CmdLEDAnimation(_leds);
        }

        // public Command updateDisabledLEDs() {
        //         System.out.println("Cmd UpdateLEDs");
        //         return new CmdLEDDefault(_leds);
        // }

        public void setTestMode(boolean inTest) {
                _armSubsystem.setTestMode(inTest);
        }

        public void setAutoMode(boolean inAuto) {
                _armSubsystem.setAutonomousMode(inAuto);
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