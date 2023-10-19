package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDs.LEDs;
import frc.robot.LEDs.LEDs.LEDAnimations;


public class CmdLEDAnimation extends CommandBase {
    private LEDs _leds;

    public CmdLEDAnimation(LEDs leds) {
        _leds = leds;
    }

    @Override
    public void initialize() {
               _leds.setAnimation(LEDAnimations.Rainbow, -1);

        System.out.println("SETTING CANDLE LED TO TEAL FOR GAME PIECE");
    }

      @Override
    public void execute() {
        // _leds.setShouldStartPartyModeAnimation(true);
        // _leds.setAnimation(LEDAnimations.Rainbow);
    }
    
    @Override
    public void end(boolean interrupted) {}
 
    @Override
    public boolean isFinished() {
        // Only run this once.
        return true;
    }
}
