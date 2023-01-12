package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    // divide by 60 to get rpm into rotations per second
    private static final double FALCON_FREE_SPEED = 6380.0; // Estimated max speed of a falcon in rmp
    private static final double DRIVE_REDUCTION = 6.75; // 6.75 : 1 reduction
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double MAX_VELOCITY = (FALCON_FREE_SPEED / 60.0) * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;

    // PID coefficients for the turning motor
    private static final double TURNING_MOTOR_P = 0.2;
    private static final double TURNING_MOTOR_I = 0;
    private static final double TURNING_MOTOR_D = 0.1;

    private static final double CANCODER_RESOLUTION = 4096;

    private WPI_TalonFX _driveMotor;
    private WPI_TalonFX _turningMotor;

    private CANCoder _turningEncoder;

    private PIDController _drivePIDController;
    private ProfiledPIDController _turningPIDController;

    private SimpleMotorFeedforward _driveFeedforward;
    private SimpleMotorFeedforward _turningFeedforward;

    public SwerveModule(int driveMotorId, int turningMotorId, int turningEncoderId, double encoderOffset) {
        _driveMotor = new WPI_TalonFX(driveMotorId);
        _turningMotor = new WPI_TalonFX(turningMotorId);

        _turningEncoder = new CANCoder(turningEncoderId);


    }

}
