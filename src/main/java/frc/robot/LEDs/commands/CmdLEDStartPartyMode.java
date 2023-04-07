package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDs.LEDs;


public class CmdLEDStartPartyMode extends CommandBase {
    private LEDs _leds;

    public CmdLEDStartPartyMode(LEDs leds) {
        _leds = leds;
    }

    @Override
    public void initialize() {
        //System.out.println("SETTING CANDLE LED TO TEAL FOR GAME PIECE");
    }

      @Override
    public void execute() {
        _leds.setShouldStartPartyModeAnimation(true);
    }
    
    @Override
    public void end(boolean interrupted) {}
 
    @Override
    public boolean isFinished() {
        // Only run this once.
        return true;
    }
}
