package frc.robot.LEDs.commands;

// import org.apache.commons.lang3.function.FailableSupplier;

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
    // Note: only used DriverStation for getting state of comms, which will likely be omitted. -KH 2023/3/27
    //private DriverStation _driverStation;
    private ArmMode armMode;
    private ArmPosition armPosition;
    private int counter;
    

    public CmdLEDDefault(LEDs leds, Arm armSubsystem) {
        _armSubsystem = armSubsystem;
        _leds = leds;
        this.counter = 0;
        
    }

    @Override
    public void initialize() {    
        
        System.out.println("LEDs Off (initializing)");
        // this._leds.brightness = 0.00;
        
    }

    @Override
    public void execute() {
        //System.out.println("Running LEDs");
        
        this.armMode = this._armSubsystem.getArmMode();
        this.armPosition = this._armSubsystem.getCurrentArmPosition();

        if (DriverStation.isDisabled()) {
            this._leds.setLEDMode(LEDMode.Off);
            System.out.println("LEDs off");
        }

        if (_leds.getRunningGamepieceAnimation()) {
            //count down for 1 sec
            this.counter = 50;
            _leds.setRunningGamepieceAnimation(false);

        }

        if (this.counter > 0) {
            System.out.println("STILL RUNNING GAME PIECE ANIMATION. Cuunter: " + counter);
            this.counter -= 1;
            this._leds.setLEDMode(LEDMode.HasGamepiece);
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
    public void end(boolean interrupted) {}
 
    @Override
    public boolean isFinished() {
        // This is a default command, so there is no end
        return false;
    }
}
