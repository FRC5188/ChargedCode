package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem_builders.Drive.Drivetrain;
import frc.robot.subsystem_builders.Drive.Odometry;

/**
 * Singleton subsystem for Drivebase.
 */
public class Drive extends SubsystemBase { 
    private Vision _visionSubsystem;

    // Holds the instance of the drive subsystem
    private static Drive _instance = null;

    private Drive(Vision visionSubsystem) {
        _visionSubsystem = visionSubsystem;
        // optional debugging parameter 
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain Info");

        Drivetrain.buildDrivebase(shuffleboardTab);
        Odometry.buildOdometry();
    }

    public static void setInstance(Vision visionSubsystem) {
        _instance = (_instance == null) ? (new Drive(visionSubsystem)) : (_instance);
    }

    public static Drive getInstance() {
        return _instance;
    }

    @Override
    public void periodic() {
        Drivetrain.updateModules();
        // Update odometry if applicable
        _visionSubsystem.getVisionEstimatedRobotPose(Odometry.getPoseEstimator());
        Odometry.update();
    }
}
        

