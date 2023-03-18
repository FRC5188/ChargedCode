package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.Arm2DPosition;
import frc.robot.subsystems.LEDs.LEDColors;
import frc.robot.subsystems.LEDs;

public class CmdLEDDefault extends CommandBase {
    private Arm _armSubsystem;
    private LEDs _leds;
    private LEDColors color;
    private ArmMode armMode;

    public CmdLEDDefault(LEDs leds, Arm armSubsystem) {
        _armSubsystem = armSubsystem;
        _leds = leds;
    }

    @Override
    public void initialize() {
        // this.color = LEDColors.White;
        // this._leds.changeColor(this.color);
        // this._leds.brightness = 0.00;
    }

    @Override
    public void execute() {
        System.out.println("Running LEDs");
        if (!DriverStation.isDSAttached()) {
            // Add this in for comp but not for home testing: || !DriverStation.isFMSAttached()
            this._leds.changeColor(LEDColors.Red);
        }
        else {
            this.armMode = this._armSubsystem.getArmMode();

            switch (armMode) {
                case Cone:
                    this._leds.changeColor(LEDColors.Yellow);
                    break;

                case Cube:
                    this._leds.changeColor(LEDColors.Purple);
                    break;


            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
 
    @Override
    public boolean isFinished() {
        // This is a default command, so there is no end
        return false;
    }
}
