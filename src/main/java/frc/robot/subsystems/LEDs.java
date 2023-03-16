package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;
//import com.ctre.phoenix.led.CANdle.LEDStripType;
//import com.ctre.phoenix.led.CANdle.VBatOutputMode;
//import .ctre.phoenix.led.ColorFlowAnimation.Direction;



public class LEDs extends SubsystemBase {
    private final CANdle _candle = new CANdle(Constants.CanIDs.CANdleID, "rio");
    private final int LedCount = 300;
    private XboxController joystick;


    public enum Colors {
        
    }
    private Colors _currentAnimation;
} 
