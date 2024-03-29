package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDs.LEDs;
//import frc.robot.LEDs.LEDs.LEDCustomAnimations;
import frc.robot.LEDs.LEDs.LEDModes;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.ArmPosition;

public class CmdLEDDefault extends CommandBase {
    private Arm _armSubsystem;
    private LEDs _leds;
    private ArmMode armMode;
    private ArmPosition armPosition;
    private int counter;
    private Double currentTemperature;
    //private ArmPosition _finalArmPosition;


    public CmdLEDDefault(LEDs leds, Arm armSubsystem) {
        _armSubsystem = armSubsystem;
        _leds = leds;
        this.counter = 0;
    }

    @Override
    public void initialize() {    

        System.out.println("LEDs Off (initializing)");
    }

    @Override
    public void execute() {
        
        this.armMode = this._armSubsystem.getArmMode();
        this.armPosition = this._armSubsystem.getCurrentArmPosition();
        this.currentTemperature = this._leds.getLEDTemperature();
        this._leds.adjustLEDTemperature(currentTemperature);
        //this._finalArmPosition = this._armSubsystem.getFinalPosition();

        if (_leds.getShouldRunGamepieceAnimation()) {
            // Code runs 50 times per second.
            // This counter should count down for one second.
            this.counter = 50;
            _leds.setShouldRunGamepieceAnimation(false);
        }

        // This is an "if" instead of an "elif."
        // Otherwise, we would skip over it when the counter is first set to 50.
        if (this.counter > 0) {
            //System.out.println("STILL RUNNING GAME PIECE ANIMATION. Counter: " + counter);
            this.counter -= 1;
            this._leds.setLEDMode(LEDModes.HasGamepiece);
        }
        
        else {
            this._leds._candle.clearAnimation(0);
            this._leds._currentAnimation = null;

            if (armPosition == ArmPosition.Stored || armPosition == ArmPosition.LoadStationPickUp || armPosition == ArmPosition.GroundPickUp || armPosition == ArmPosition.EnGarde) {
                switch (armMode) {
                    case Cone:
                        this._leds.setLEDMode(LEDModes.Cone);
                        break;

                    case Cube:
                        this._leds.setLEDMode(LEDModes.Cube);
                        break;
                }
            }

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
        }
    }

    @Override
    public void end(boolean interrupted) {}
 
    @Override
    public boolean isFinished() {
        // This is a default command, so there is no end.
        return false;
    }
}
