package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleLEDS extends SubsystemBase {
    public void setLEDS(short[] rgbVaue, short brightness) { //This is a set up which takes an Enumeration and a Brightness level that spits out an RGB Triplet.
        this._candle.setLEDs(rgbVaue[0], rgbVaue[1], rgbVaue[2]);
        this._candle.configBrightnessScalar(brightness); 
    }
    private final CANdle _candle = new CANdle(Constants.CanIDs.CANdleID, "rio");
    
    public CANdleLEDS(){
        _candle.configFactoryDefault();
        _candle.configLOSBehavior(false); 
    }

    public void turnOnLEDS() {
        _candle.setLEDs(255, 255, 255); 
        _candle.configBrightnessScalar(1.0); 
    }
    public void configBrightness(double percent) {
         _candle.configBrightnessScalar(percent); 
        }
    public void turnOffLEDS() {
        _candle.configBrightnessScalar(0.0); 
    }

    public double getVbat() { return _candle.getBusVoltage(); }
    public double get5V() { return _candle.get5VRailVoltage(); }
    public double getCurrent() { return _candle.getCurrent(); }
    public double getTemperature() { return _candle.getTemperature(); }
    public void configLos(boolean disableWhenLos) { _candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { _candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { _candle.configStatusLedState(offWhenActive, 0); }
}
