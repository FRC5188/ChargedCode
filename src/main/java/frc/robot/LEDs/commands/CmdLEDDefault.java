package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDs.LEDs;
//import frc.robot.LEDs.LEDs.LEDAnimations;
//import frc.robot.LEDs.LEDs.LEDAnimations;
//import frc.robot.LEDs.LEDs.LEDCustomAnimations;
import frc.robot.LEDs.LEDs.LEDModes;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
//import frc.robot.arm.Arm.ArmPosition;
import edu.wpi.first.wpilibj.DriverStation;

public class CmdLEDDefault extends CommandBase {
    private Arm _armSubsystem;
    private LEDs _leds;
    private ArmMode armMode;
    //private ArmPosition armPosition;
    private int gamepieceCounter;
    private Double currentTemperature;
    //private ArmPosition _finalArmPosition;


    public CmdLEDDefault(LEDs leds, Arm armSubsystem) {
        _armSubsystem = armSubsystem;
        _leds = leds;
        this.gamepieceCounter = 0;
        //addRequirements(_leds);
    }

    @Override
    public void initialize() {    

        //System.out.println("LEDs Off (initializing)");
        this._leds.setLEDMode(LEDModes.LostGamepiece);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
     }

    @Override
    public void execute() {
        
        if (DriverStation.isDisabled()) {
            _leds.setLEDMode(LEDModes.PartyMode);
            _leds._candle.configBrightnessScalar(0.1);
            System.out.println("########## Disabled! Activate rainbow. ##########");
            return;
        }

        this.armMode = this._armSubsystem.getArmMode();
        //this.armPosition = this._armSubsystem.getCurrentArmPosition();
        this.currentTemperature = this._leds.getLEDTemperature();
        this._leds.adjustLEDTemperature(currentTemperature);
        //this._finalArmPosition = this._armSubsystem.getFinalPosition();

        if (_leds.getShouldRunGamepieceAnimation()) {
            // Code runs 50 times per second.
            // This counter should count down for one second.
            this.gamepieceCounter = 50;
            _leds.setShouldRunGamepieceAnimation(false);
        }

        // This is an "if" instead of an "elif."
        // Otherwise, we would skip over it when the counter is first set to 50.
        if (this.gamepieceCounter > 0) {
            //System.out.println("STILL RUNNING GAME PIECE ANIMATION. Counter: " + counter);
            this.gamepieceCounter -= 1;
            this._leds.setLEDMode(LEDModes.HasGamepiece);
        }
        
        else {
            //System.out.println("*********Else*********");
            this._leds._candle.clearAnimation(0);
            //this._leds._candle.configBrightnessScalar(0.5);
            //this._leds._currentAnimation = null;

            //if (armPosition == ArmPosition.Stored || armPosition == ArmPosition.LoadStationPickUp || armPosition == ArmPosition.GroundPickUp || armPosition == ArmPosition.EnGarde) {
                switch (armMode) {

                    case Cone:
                        this._leds.setLEDMode(LEDModes.Cone);
                        break;

                    case Cube:
                        //System.out.println("Running PartyMode");
                        this._leds.setLEDMode(LEDModes.Cube);
                        // if (this._leds._currentAnimation == LEDAnimations.TealTwinkle) {
                        //     this._leds._candle.clearAnimation(0);
                        //     return;
                        // }
                        // else {
                        //     this._leds.setAnimation(LEDAnimations.TealTwinkle); 
                        // }
                        break;
                }
            
        
            //}

            // else {
            //     switch (_finalArmPosition) {
            //         case High:
            //             this._leds.setLEDMode(LEDModes.Off);
            //             this._leds.setCustomAnimation(LEDCustomAnimations.High);
            //             break;
            //         case Middle:
            //             this._leds.setLEDMode(LEDModes.Off);
            //             this._leds.setCustomAnimation(LEDCustomAnimations.Mid);
            //             break;
            //         case Stored:
            //             this._leds.setLEDMode(LEDModes.Off);
            //             this._leds.setCustomAnimation(LEDCustomAnimations.Low);
            //             break;
            //         default:
            //             break;

            //     }
            // } 
            //}

            
        }

        //_leds.setLEDMode(LEDModes.Off);
    }

    @Override
    public void end(boolean interrupted) {}
 
    @Override
    public boolean isFinished() {
        // This is a default command, so there is no end.
        return false;
    }
}
