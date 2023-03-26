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
        Teal,
        Yellow,
        Purple,
        White,
        Red,
        Green, 
        Blue,
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

    // TODO: Change to length of LED strips. Only necessary for animations, so it's working for now. -KH
    // private final int LEDCount = 9;

    public CANdle candle = new CANdle(Constants.CanIDs.CANDLE_ID, "rio");
    private LEDMode currentMode = LEDMode.Default;
    public Boolean _runningHasGamepieceAnimation = false;

    public LEDs() {

        //TODO: Test brightness and update this to preferred look. -KH
        candle.configBrightnessScalar(0.5);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false; //don't turn off leds when we lose comms. let us decide
        configAll.stripType = LEDStripType.RGB; //normal RBG strips.
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);

    }



    public Boolean getRunningGamepieceAnimation() {
        return _runningHasGamepieceAnimation;
    }

    public void setRunningGamepieceAnimation(Boolean runningHasGamepieceAnimation) {
        _runningHasGamepieceAnimation = runningHasGamepieceAnimation;
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
                setColor(LEDColors.Teal);
                this.currentMode = LEDMode.HasGamepiece;
                break;

            case IsAligned:
                setColor(LEDColors.Blue);
                this.currentMode = LEDMode.IsAligned;
                break;

            // Note: the following scoring colors are arbitrary/temporary and can be changed to fit driver preferences.
            // TODO: Decide whether to have separate scoring cone + cube LEDModes.

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
                candle.setLEDs(210, 55, 120); 
                break;

            case Teal: 
                candle.setLEDs(2, 153, 138);
                break;

            case Yellow:
                candle.setLEDs(255, 213, 0);
                break;

            case Purple:
                candle.setLEDs(165, 0, 215);
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

            case Blue:
                candle.setLEDs(0, 0, 255);
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
