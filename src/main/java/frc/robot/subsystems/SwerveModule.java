package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    // divide by 60 to get rpm into rotations per second
    private static final double FALCON_FREE_SPEED = 6380.0; // Estimated max speed of a falcon in rmp
    private static final double DRIVE_REDUCTION = 6.75; // 6.75 : 1 reduction
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

    public static final double MAX_VELOCITY = (FALCON_FREE_SPEED / 60.0) * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
    // Lower the max voltage to reduce robot speed
    public static final double MAX_VOLTAGE = 12.0;

    // PID coefficients for the turning motor
    private static final double TURNING_MOTOR_P = 0.2;
    private static final double TURNING_MOTOR_I = 0;
    private static final double TURNING_MOTOR_D = 0.1;

    public final static double kNeutralDeadband = 0.001;
    public final static int CANCoderDataPeriod = 20;

    private WPI_TalonFX _driveMotor;
    private WPI_TalonFX _turningMotor;

    private CANCoder _turningEncoder;

    private PIDController _turningPIDController;

    public SwerveModule(int driveMotorId, int turningMotorId, int turningEncoderId, double encoderOffset) {
        _driveMotor = new WPI_TalonFX(driveMotorId);
        _turningMotor = new WPI_TalonFX(turningMotorId);
        _turningEncoder = new CANCoder(turningEncoderId);

        _turningPIDController = new PIDController(TURNING_MOTOR_P, TURNING_MOTOR_I, TURNING_MOTOR_D);
        
        _driveMotor.configFactoryDefault();
        _turningMotor.configFactoryDefault();
        _turningEncoder.configFactoryDefault();

        _driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        _turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        _driveMotor.setNeutralMode(NeutralMode.Coast);
        _turningMotor.setNeutralMode(NeutralMode.Brake);

        _driveMotor.setSensorPhase(false);
        _turningMotor.setSensorPhase(false);

        _driveMotor.setInverted(TalonFXInvertType.CounterClockwise);
        _turningMotor.setInverted(TalonFXInvertType.Clockwise);

        _driveMotor.configNeutralDeadband(kNeutralDeadband);
        _turningMotor.configNeutralDeadband(kNeutralDeadband);

        // _driveMotor.setStatusFramePeriod(1, 20);
        // _turningMotor.setStatusFramePeriod(1, 20);
        _turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, CANCoderDataPeriod);
    }

    public double getTurningAngleRadians() {
        return Math.toRadians(_turningEncoder.getAbsolutePosition());
    }

    public void set(SwerveModuleState moduleInfo) {
        _driveMotor.setVoltage(moduleInfo.speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE);
        _turningMotor.setVoltage(_turningPIDController.calculate(getTurningAngleRadians(), moduleInfo.angle.getRadians()));
    }
}
