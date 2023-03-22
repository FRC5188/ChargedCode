package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDs.LEDs;
import frc.robot.LEDs.LEDs.LEDMode;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.ArmPosition;

public class CmdLEDDefault extends CommandBase {
    private Arm _armSubsystem;
    private LEDs _leds;
    private DriverStation _driverStation;
    private ArmMode armMode;
    private ArmPosition armPosition;

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
        //System.out.println("Running LEDs");
        
        this.armMode = this._armSubsystem.getArmMode();
        this.armPosition = this._armSubsystem.getCurrentArmPosition();
        // this.isRobotDisabled = this._driverStation.isDisabled();
        if (DriverStation.isDisabled()) {
            this._leds.setLEDMode(LEDMode.Off);
        }
        
        else {
            if (armPosition == ArmPosition.Stored || armPosition == ArmPosition.LoadStationPickUp || armPosition == ArmPosition.GroundPickUp || armPosition == ArmPosition.EnGarde) {
                switch (armMode) {
                    case Cone:
                        this._leds.setLEDMode(LEDMode.Cone);
                        break;

                    case Cube:
                        this._leds.setLEDMode(LEDMode.Cube);
                        break;

                }
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
