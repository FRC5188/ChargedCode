package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CmdDriveAuto;
import frc.robot.commands.CmdDriveWithJoystick;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private Drive _driveSubsystem;

  private CommandXboxController _driveController;

  public RobotContainer() {
    _driveSubsystem = new Drive();

    _driveController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //_driveSubsystem.setDefaultCommand(new CmdDriveWithJoystick(_driveSubsystem, () -> _driveController.getLeftY(), () -> _driveController.getLeftX(), () -> _driveController.getRightX()));
    _driveController.a().onTrue(new CmdDriveAuto(_driveSubsystem, 1, 1, 0));
  }
}
