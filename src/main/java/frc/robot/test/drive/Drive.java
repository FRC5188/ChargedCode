package frc.robot.test.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.test.console_output.Output;

@Deprecated
public class Drive {
    private static frc.robot.drive.Drive subsystem;

    public static Command runFowardDrivetrainCheck(){
        final long millisecondsToRun = 2000; // 2 seconds
        return new CommandBase(){
            @Override
            public void execute(){
                double currentSpeed = 0.1;
                while (currentSpeed <= 1.0){
                    long initTime = RobotController.getFPGATime();
                    Output.information("Drive Speed: " + currentSpeed);
                    while (RobotController.getFPGATime() - initTime <= millisecondsToRun){
                        subsystem.drive(
                            new ChassisSpeeds(
                                (currentSpeed * frc.robot.drive.Drive.MAX_VELOCITY_METERS_PER_SECOND * subsystem.getSpeedMultiplier()),
                                0,
                                0
                            )
                        );
                    }
                    currentSpeed += 0.1;
                }
            };
        };
    }

    public static Command runBackwardDrivetrainCheck(){
        final long millisecondsToRun = 2000; // 2 seconds
        return new CommandBase(){
            @Override
            public void execute(){
                double currentSpeed = -0.1;
                while (currentSpeed >= -1.0){
                    long initTime = RobotController.getFPGATime();
                    Output.information("Drive Speed: " + currentSpeed);
                    while (RobotController.getFPGATime() - initTime <= millisecondsToRun){
                        subsystem.drive(
                            new ChassisSpeeds(
                                (currentSpeed * frc.robot.drive.Drive.MAX_VELOCITY_METERS_PER_SECOND * subsystem.getSpeedMultiplier()),
                                0,
                                0
                            )
                        );
                    }
                    currentSpeed -= 0.1;
                }
            };
        };
    }

    public static Command runRotationModuleCheck(){
        final long millisecondsToRun = 2000; // 2 seconds
        return new CommandBase(){
            @Override
            public void execute(){
                double currentRotationalSpeed = 0.1;
                while (currentRotationalSpeed <= 1.0){
                    long initTime = RobotController.getFPGATime();
                    Output.information("Rotational Speed: " + currentRotationalSpeed);
                    while (RobotController.getFPGATime() - initTime <= millisecondsToRun){
                        subsystem.drive(
                            new ChassisSpeeds(
                                0,
                                0,
                                (currentRotationalSpeed * frc.robot.drive.Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * subsystem.getSpeedMultiplier())
                            )
                        );
                    }
                    currentRotationalSpeed += 0.1;
                }
            };
        };
    }
    /// Runs all checks for the subsystem sequentially. 
    public static Command runChecks(){
        return null;
    }
}