package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.LEDs.LEDs;
import frc.robot.LEDs.LEDs.LEDAnimations;

public class CmdLEDSetAnimation extends InstantCommand {

    /**
     * 
     * @param animation - LEDAnimations enum
     * @param duration - time animation runs for in seconds (-1 if forever)
     * @param leds - LED subsystem
     */
    public void CmdLEDAnimation(LEDAnimations animation, int duration, LEDs leds) {

        // TODO: Include duration - KtH 2023/10/12
        leds.setAnimation(animation, duration);
    }
 
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
