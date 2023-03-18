package frc.robot.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;

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

    public LEDs() {

        candle.configBrightnessScalar(0.4);
    
        //init: set to white
        candle.setLEDs(255, 255, 255);
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