package frc.robot.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

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

    public enum LEDAnimations {
        Rainbow,
        Fire,

        PinkStrobe,

        TealStrobe,
        TealTwinkle
    }

    // TODO: Change to length of LED strips. Only necessary for animations, so it's working for now. -KH 2023/3/27
    // TODO: Find out if the length includes the 8 LEDs on the Candle. If so, include in a comment. -KH 2023/3/27

    private final int LEDCount = 9;

    public CANdle _candle = new CANdle(Constants.CanIDs.CANDLE_ID, "rio");
    private LEDMode _currentMode = LEDMode.Default;
    public Boolean _runningHasGamepieceAnimation = false;
    private LEDAnimations _currentAnimation = null;
    private Animation _storedAnimation;

    public LEDs() {

        //TODO: Test brightness and update this to preferred look. -KH 2023/3/27
        _candle.configBrightnessScalar(0.5);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false; //don't turn off leds when we lose comms. let us decide
        configAll.stripType = LEDStripType.RGB; //normal RBG strips.
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        _candle.configAllSettings(configAll, 100);

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
     * @param mode - Input enumeration name from LEDMode (ex: HasGamepiece)
     */

    public void setLEDMode(LEDMode mode) {
        switch(mode) {

            case LostComms:
                setColor(LEDColors.Red);
                this._currentMode = LEDMode.LostComms;
                break;

            case Cone:
                setColor(LEDColors.Yellow);
                this._currentMode = LEDMode.Cone;
                break;

            case Cube:
                setColor(LEDColors.Purple);
                this._currentMode = LEDMode.Cube;
                break;

            case HasGamepiece:
                setAnimation(LEDAnimations.TealStrobe);
                this._currentMode = LEDMode.HasGamepiece;
                break;

            case IsAligned:
                setColor(LEDColors.Blue);
                this._currentMode = LEDMode.IsAligned;
                break;

            // Note: the following scoring colors are arbitrary/temporary and can be changed to fit driver preferences.
            // TODO: Decide whether to have separate scoring cone + cube LEDModes. -KH 2023/3/27

            case ScoreHigh:
                setColor(LEDColors.White);
                this._currentMode = LEDMode.ScoreHigh;
                break;

            case ScoreMid:
                setColor(LEDColors.White);
                this._currentMode = LEDMode.ScoreMid;
                break;

            case ScoreLow:
                setColor(LEDColors.White);
                this._currentMode = LEDMode.ScoreLow;
                break;

            case Off:
                setColor(LEDColors.Off);
                this._currentMode = LEDMode.Off;
                break;

            default:
                setColor(LEDColors.Pink);
                this._currentMode = LEDMode.Default;

        }
    }

    /**
     * Changes the color of the LEDs. Called by setLEDMode command.
     * 
     * @param color - Input enumeration name from LEDColors (ex: White)
     */

    public void setColor(LEDColors color) {

        switch(color) {
            case Pink: 
                _candle.setLEDs(210, 55, 120); 
                break;

            case Teal: 
                _candle.setLEDs(2, 153, 138);
                break;

            case Yellow:
                _candle.setLEDs(255, 213, 0);
                break;

            case Purple:
                _candle.setLEDs(165, 0, 215);
                break;
            
            case White:
                _candle.setLEDs(255, 255, 255);
                break;

            case Red:
                _candle.setLEDs(255, 0, 0);
                break;

            case Green:
                _candle.setLEDs(0, 255, 0);
                break;

            case Blue:
                _candle.setLEDs(0, 0, 255);
                break;

            case Off:
                _candle.setLEDs(0, 0, 0);
                _candle.configBrightnessScalar(0.0, 100);
                break;

            default:
                _candle.setLEDs(255, 255, 255);
        }
    }

    public void setAnimation(LEDAnimations animation) {

        _currentAnimation = animation;

        switch(animation) {

            case PinkStrobe: 
                _storedAnimation = new StrobeAnimation(210, 55, 120, 0, 0, LEDCount);
                break;

            case Rainbow: 
                _storedAnimation = new RainbowAnimation(0.4, 0.1, LEDCount);
                break;

            case Fire: 
                _storedAnimation = new FireAnimation(0.5, 0.7, LEDCount, 0.7, 0.5);
                break;

            case TealStrobe:
                _storedAnimation = new StrobeAnimation(2, 153, 138, 0, 0, LEDCount);
                break;

            case TealTwinkle:
                _storedAnimation = new TwinkleAnimation(2, 153, 138, 0, 0.4, LEDCount, TwinklePercent.Percent6);
                break;

            default:
                _storedAnimation = null;
                break;

        }

        _candle.animate(_storedAnimation, LEDCount);

        // Use this print statement for testing/debugging:
        //System.out.println("Current animation: " + _storedAnimation.toString());
    }
}