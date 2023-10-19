package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.LEDs.LEDs;
import frc.robot.LEDs.LEDs.LEDColors;

public class CmdLEDSetColor extends InstantCommand {

    public void CmdLEDAnimation(LEDColors color, LEDs leds) {
        leds.setColor(color);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}