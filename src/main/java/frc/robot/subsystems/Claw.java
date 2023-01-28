package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    private CANSparkMax _intakeMotor;
    public void setSpeed(double speed){
        _intakeMotor.set(speed);
    }
    public double getChangeInCurrent(){
        return 0;
    }
}
