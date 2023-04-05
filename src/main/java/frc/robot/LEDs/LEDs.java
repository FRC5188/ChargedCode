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
//import com.ctre.phoenix.led.SingleFadeAnimation;

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

    public enum LEDModes {
        Default,
        Cone,
        Cube,
        HasGamepiece,
        IsAligned,
        ScoreHigh,
        ScoreMid,
        ScoreLow,
        LostGamepiece,
        PartyMode,
        Off
    }

    public enum LEDAnimations {
        Rainbow,
        Fire,

        PinkStrobe,

        TealStrobe,
        TealTwinkle,
        PinkPartyMode
    }

    public enum LEDCustomAnimations {
        High,
        Mid,
        Low
    }

    // Includes the 8 LEDs on the Candle and 28 in each LED strip.
    private final int LEDCount = 64;

    private final int PinkRValue = 255;
    private final int PinkGValue = 150;
    private final int PinkBValue = 220;

    private final int WhiteRValue = 255;
    private final int WhiteGValue = 255;
    private final int WhiteBValue = 255; 

    private final int numCycles = 50;

    private int PinkToWhiteRIncrement = (WhiteRValue - PinkRValue)/numCycles*-1;
    private int PinkToWhiteGIncrement = (WhiteGValue - PinkGValue)/numCycles*-1;
    private int PinkToWhiteBIncrement = (WhiteBValue - PinkBValue)/numCycles*-1;

    public int _currentRValue = 255;
    public int _currentGValue = 150;
    public int _currentBValue = 220;

    public CANdle _candle = new CANdle(Constants.CanIDs.CANDLE_ID, "rio");
    public Boolean _shouldRunHasGamepieceAnimation = false;
    private Animation _storedAnimation;

    //TODO: Should I set LEDMode to default or null at first? -KtH 2023/3/27
    public LEDModes _currentMode = LEDModes.Default;
    public LEDColors _currentColor;
    public LEDAnimations _currentAnimation = null;
    private LEDCustomAnimations _currentCustomAnimation;

    public LEDs() {

        //TODO: Test brightness and update this to preferred look. -KtH 2023/3/27
        _candle.configBrightnessScalar(0.5);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false; //don't turn off leds when we lose comms. let us decide
        configAll.stripType = LEDStripType.RGB; //normal RBG strips.
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        _candle.configAllSettings(configAll, 100);

    }

    public Boolean getShouldRunGamepieceAnimation() {
        return _shouldRunHasGamepieceAnimation;
    }

    public void setShouldRunGamepieceAnimation(Boolean shouldRunHasGamepieceAnimation) {
        _shouldRunHasGamepieceAnimation = shouldRunHasGamepieceAnimation;
    }
    
    /**
     * Maps a mode to an LED sequence/color. Called by LEDDefault command.
     * 
     * @param mode - Input enumeration name from LEDMode (ex: HasGamepiece)
     */

    public void setLEDMode(LEDModes mode) {

        this._candle.configBrightnessScalar(0.5);

        switch(mode) {

            case LostGamepiece:
                setColor(LEDColors.Red);
                this._currentMode = LEDModes.LostGamepiece;
                break;

            case Cone:
                setColor(LEDColors.Yellow);
                this._currentMode = LEDModes.Cone;
                break;

            case Cube:
                setColor(LEDColors.Purple);
                this._currentMode = LEDModes.Cube;
                break;

            case HasGamepiece:
                setAnimation(LEDAnimations.TealStrobe);
                this._currentMode = LEDModes.HasGamepiece;
                break;

            case IsAligned:
                setColor(LEDColors.Blue);
                this._currentMode = LEDModes.IsAligned;
                break;

            // Note: the following scoring colors are arbitrary/temporary and can be changed to fit driver preferences.
            // TODO: Decide whether to have separate scoring cone + cube LEDModes. -KtH 2023/3/27

            case ScoreHigh:
                setColor(LEDColors.White);
                this._currentMode = LEDModes.ScoreHigh;
                break;

            case ScoreMid:
                setColor(LEDColors.White);
                this._currentMode = LEDModes.ScoreMid;
                break;

            case ScoreLow:
                setColor(LEDColors.White);
                this._currentMode = LEDModes.ScoreLow;
                break;

            case PartyMode:
                setAnimation(LEDAnimations.PinkPartyMode);
                this._currentMode = LEDModes.PartyMode;
                break;

            case Off:
                setColor(LEDColors.Off);
                this._currentMode = LEDModes.Off;
                break;

            default:
                setColor(LEDColors.Pink);
                this._currentMode = LEDModes.Default;
                break;
        }
    }

    /**
     * Changes the color of the LEDs. Called by setLEDMode method.
     * 
     * @param color - Input enumeration name from LEDColors (ex: White)
     */

    public void setColor(LEDColors color) {

        switch(color) {
            case Pink: 
                _candle.setLEDs(210, 55, 120); 
                this._currentColor = LEDColors.Pink;
                break;

            case Teal: 
                _candle.setLEDs(2, 153, 138);
                this._currentColor = LEDColors.Teal;
                break;

            case Yellow:
                _candle.setLEDs(255, 213, 0);
                this._currentColor = LEDColors.Yellow;
                break;

            case Purple:
                _candle.setLEDs(165, 0, 215);
                this._currentColor = LEDColors.Purple;
                break;
            
            case White:
                _candle.setLEDs(255, 255, 255);
                this._currentColor = LEDColors.White;
                break;

            case Red:
                _candle.setLEDs(255, 0, 0);
                this._currentColor = LEDColors.Red;
                break;

            case Green:
                _candle.setLEDs(0, 255, 0);
                this._currentColor = LEDColors.Green;
                break;

            case Blue:
                _candle.setLEDs(0, 0, 255);
                this._currentColor = LEDColors.Blue;
                break;

            case Off:
                _candle.setLEDs(0, 0, 0);
                _candle.configBrightnessScalar(0.0, 100);
                this._currentColor = LEDColors.Off;
                break;

            default:
                _candle.setLEDs(255, 255, 255);
                this._currentColor = LEDColors.White;
        }
    }

    /**
     * Gives an animation to the LEDs. Called by setLEDMode method.
     * 
     * @param animation - Input enumeration name from LEDAnimations (ex: TealStrobe)
     * 
     */

    public void setAnimation(LEDAnimations animation) {

        this._candle.clearAnimation(0);

        switch(animation) {

            case PinkStrobe: 
                this._storedAnimation = new StrobeAnimation(210, 55, 120, 0, 0, LEDCount);
                break;

            case Rainbow: 
                this._storedAnimation = new RainbowAnimation(0.4, 0.1, LEDCount);
                break;

            case Fire: 
                this._storedAnimation = new FireAnimation(0.5, 0.7, LEDCount, 0.7, 0.5);
                break;

            case TealStrobe:
                this._storedAnimation = new StrobeAnimation(2, 153, 138, 0, 0, LEDCount);
                break;

            case TealTwinkle:
                this._storedAnimation = new TwinkleAnimation(2, 153, 138, 0, 0.4, LEDCount, TwinklePercent.Percent6);
                break;

            case PinkPartyMode:
                this._storedAnimation = new TwinkleAnimation(_currentRValue, _currentGValue, _currentBValue, 100, 0.6, LEDCount, TwinklePercent.Percent64);

                if (_currentRValue == WhiteRValue && _currentGValue == WhiteGValue && _currentBValue == WhiteBValue) {
                    PinkToWhiteRIncrement *= -1;
                    PinkToWhiteGIncrement *= -1;
                    PinkToWhiteBIncrement *= -1;
                }

                else if (_currentRValue == PinkRValue && _currentGValue == PinkGValue && _currentBValue == PinkBValue) {
                    PinkToWhiteRIncrement *= -1;
                    PinkToWhiteGIncrement *= -1;
                    PinkToWhiteBIncrement *= -1;
                }

                _currentRValue += PinkToWhiteRIncrement;
                _currentGValue += PinkToWhiteGIncrement;
                _currentBValue += PinkToWhiteBIncrement;
                break;

            default:
                this._storedAnimation = null;
                break;

        }

        this._candle.animate(this._storedAnimation, 0);

        // Use this print statement for testing/debugging:
        //System.out.println("Current animation: " + _storedAnimation.toString());

        this._currentAnimation = animation;
    }

    public void setCustomAnimation(LEDCustomAnimations customAnimation) {
        this._currentCustomAnimation = customAnimation;

        switch(customAnimation) {
            case High:
                setColor(LEDColors.Pink);
                break;

            case Mid:
                this._candle.setLEDs(210, 55, 120, 100, 0, LEDCount/3);
                this._candle.setLEDs(210, 55, 120, 100, LEDCount*2/3, LEDCount/3);
                break;

            case Low:
                // TODO: Should I fix white value? I put in an arbitrary value, but will it count for an RGB strip? -KtH 2023/3/28
                this._candle.setLEDs(210, 55, 120, 100, 8, LEDCount/6);
                this._candle.setLEDs(210, 55, 120, 100, (LEDCount*5/6) + 8, LEDCount/6);
                break;

        }
    }

    /**
     * Returns enumeration from LEDModes.
     */

    public LEDModes getCurrentLEDMode() {
        return this._currentMode;
    }

    /**
     * Returns enumeration from LEDAnimations.
     */

    public LEDAnimations getCurrentLEDAnimation() {
        return this._currentAnimation;
    }

    /**
     * Returns enumeration from LEDColors.
     */

    public LEDColors getCurrentLEDColor() {
        return this._currentColor;
    }

    /**
     * Returns enumeration from LEDCustomAnimations.
     */

    public LEDCustomAnimations getCurrentLEDCustomAnimation() {
        return this._currentCustomAnimation;
    }

    /**
     * @return Double - temperature in Celsius
     */

    public Double getLEDTemperature() {
        return this._candle.getTemperature();
    }

    /**
     * If LEDs are greater that 80 degrees Celsius, turn them off. Prevents vision (which is connected) from failing.
     * Called by LEDDefault command.
     * 
     * @param temperature - Input Double for temperature (ex: currentTemperature)
     */

    public void adjustLEDTemperature(Double temperature) {
        if (temperature >= 80.0) {
            this._candle.setLEDs(0, 0, 0);
            this._candle.configBrightnessScalar(0.0);
        }
    }
}