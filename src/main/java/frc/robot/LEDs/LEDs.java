package frc.robot.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
//import com.ctre.phoenix.led.StrobeAnimation;

public class LEDs extends SubsystemBase {

    public enum LEDColors {
        Pink,
        Blue,
        Yellow,
        Purple,
        White,
        Red,
        Green, 
        Off
    }

    public enum LEDMode {
        Default,
        Cone,
        Cube,
        HasGamepiece,
        IsAligned,
        ScoreHigh,
        ScoreMid,
        ScoreLow,
        LostComms,
        Off
    }

    // TODO: Change to length of LED strips.
    private final int LEDCount = 8;
    public CANdle candle = new CANdle(Constants.CanIDs.CANDLE_ID, "rio");
    private LEDMode currentMode = LEDMode.Default;

    public LEDs() {

        //TODO: Test brightness and update this to preferred look.
        candle.configBrightnessScalar(0.5);
        
    
        // init: set to white
        candle.setLEDs(255, 255, 255);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false; //don't turn off leds when we lose comms. let us decide
        configAll.stripType = LEDStripType.RGB; //normal RBG strips.
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);

    }
    
    /**
     * Maps a mode to an LED sequence/color. Called by LEDDefault command.
     * 
     * @param mode Input enumeration name from LEDMode (ex: HasGamepiece)
     */

//TODO: Finish switch case
    public void setLEDMode(LEDMode mode) {
        switch(mode) {

            case LostComms:
                setColor(LEDColors.Red);
                this.currentMode = LEDMode.LostComms;
                break;

            case Cone:
                setColor(LEDColors.Yellow);
                this.currentMode = LEDMode.Cone;
                break;

            case Cube:
                setColor(LEDColors.Purple);
                this.currentMode = LEDMode.Cube;
                break;

            case HasGamepiece:
                setColor(LEDColors.Green);
                this.currentMode = LEDMode.HasGamepiece;
                break;

            case IsAligned:
                setColor(LEDColors.Blue);
                this.currentMode = LEDMode.IsAligned;
                break;

            // Note: the following scoring colors are arbitrary/temporary and can be changed to fit driver preferences.
            // TODO: Decide whether to have separate cone + cube LEDModes.

            case ScoreHigh:
                setColor(LEDColors.White);
                this.currentMode = LEDMode.ScoreHigh;
                break;

            case ScoreMid:
                setColor(LEDColors.White);
                this.currentMode = LEDMode.ScoreMid;
                break;

            case ScoreLow:
                setColor(LEDColors.White);
                this.currentMode = LEDMode.ScoreLow;
                break;

            case Off:
                setColor(LEDColors.Off);
                this.currentMode = LEDMode.Off;
                break;
            default:
                setColor(LEDColors.Pink);
                this.currentMode = LEDMode.Default;

        }
    }

    /**
     * Changes the color of the LEDs. Called by setLEDMode command.
     * 
     * @param color Input enumeration name from LEDColors (ex: White)
     */

    public void setColor(LEDColors color) {

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

            case Green:
                candle.setLEDs(0, 255, 0);
                break;

            case Off:
                candle.setLEDs(0, 0, 0);
                candle.configBrightnessScalar(0.0, 100);
                break;

            default:
                candle.setLEDs(255, 255, 255);
        }
    }

    //private Colors _currentAnimation;
}

// TODO: Revisit auto/teleop inits. Should LEDs start there?