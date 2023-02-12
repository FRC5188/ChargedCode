package frc.robot.subsystem_builders.Drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public abstract class Odometry {
    private static SwerveDrivePoseEstimator _poseEstimator;

    public static void buildOdometry(){
        Gyroscope.buildGyroscope();
        _poseEstimator = new SwerveDrivePoseEstimator(Drivetrain.getKinematics(), Gyroscope.getGyroscopeRotation(), new SwerveModulePosition[]{
                    Drivetrain.getFrontLeftModule().getModulePosition(), Drivetrain.getFrontLeftModule().getModulePosition(),
                    Drivetrain.getBackLeftModule().getModulePosition(), Drivetrain.getBackRightModule().getModulePosition()},
                    new Pose2d(),
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
        Gyroscope.reset();
    }
    public static Pose2d getPose(){return _poseEstimator.getEstimatedPosition();}
    public static Rotation2d getOdometryRotation2d(){return getPose().getRotation();}
    public static void zeroGyroscope(){
            Gyroscope.zeroYaw();
            _poseEstimator.resetPosition(Gyroscope.getGyroscopeRotation(), null, null);
    }
    public static SwerveDrivePoseEstimator getPoseEstimator(){return _poseEstimator;}
    public static void update(){
            _poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getOdometryRotation2d(), new SwerveModulePosition[]{
                    Drivetrain.getFrontLeftModule().getModulePosition(), Drivetrain.getFrontLeftModule().getModulePosition(),
                    Drivetrain.getBackLeftModule().getModulePosition(), Drivetrain.getBackRightModule().getModulePosition()});
    }
    public static void resetPosition(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions, Pose2d initialHolonomicPosition){
        _poseEstimator.resetPosition(gyroRotation, modulePositions, initialHolonomicPosition);
    }
}