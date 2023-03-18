package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;
//import com.ctre.phoenix.led.CANdle.LEDStripType;
//import com.ctre.phoenix.led.CANdle.VBatOutputMode;
//import .ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDs extends SubsystemBase {



    // TODO: We're not using the enum yet. Keep it?
    public enum LEDColors {
        Pink,
        Blue,
        Yellow,
        Purple,
        White,
        Red
    }

    public CANdle candle = new CANdle(Constants.CanIDs.CANDLE_ID, "rio");
    public double brightness = 0.0;

    //TODO: Find the number of LEDs.
    //private final int LedCount = 300;
    //private XboxController joystick;

    public LEDs() {

        candle.configBrightnessScalar(0.4);
    
        //init: set to white
        candle.setLEDs(255, 255, 255);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false; //dont turn off leds when we lose comms. let us decide
        configAll.stripType = LEDStripType.RGB; //normal RBG strips
        configAll.brightnessScalar = 1;  //
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);

    }

    /**
     * Changes the color of the LEDs. Called by LEDDefault command.
     * @param color Input enumeration name from LEDColors (ex: White)
     */
    public void changeColor(LEDColors color) {
        switch(color) {
            case Pink: 
                candle.setLEDs(205, 55, 130); 
                break;

            case Blue: 
                candle.setLEDs(0, 0, 255);
                break;

            case Yellow:
                candle.setLEDs(235, 225, 0);
                break;

            case Purple:
                candle.setLEDs(185, 0, 205);
                break;
            
            case White:
                candle.setLEDs(255, 255, 255);
                break;

            case Red:
                candle.setLEDs(255, 0, 0);
                break;

            default:
                candle.setLEDs(255, 255, 255);
        }
    }

    //private Colors _currentAnimation;
}

// TODO: Revisit auto/teleop inits. Should LEDs start there?