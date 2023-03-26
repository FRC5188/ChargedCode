package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDs.LEDs;
import frc.robot.LEDs.LEDs.LEDMode;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmMode;
import frc.robot.arm.Arm.ArmPosition;

public class CmdLEDPieceCollected extends CommandBase {
    private Arm _armSubsystem;
    private LEDs _leds;
    // private DriverStation _driverStation;
    // private ArmMode armMode;
    private ArmPosition armPosition;
    private Timer _timer;

    public CmdLEDPieceCollected(LEDs leds, Arm armSubsystem) {
        _armSubsystem = armSubsystem;
        _leds = leds;
        _timer = new Timer();
    }

    @Override
    public void initialize() {
        // print statement here
        System.out.println("SETTING CANDLE LED TO GREEN FOR GAME PIECE");
        
    }

      @Override
    public void execute() {
        _leds.setRunningGamepieceAnimation(true);
        // if (armPosition == ArmPosition.LoadStationPickUp || armPosition == ArmPosition.GroundPickUp){
        //     if (_armSubsystem.intakeHasPiece()) {
        //         // if (Arm.checkGamepiece()) {
        //         _timer.start();
        //         this._leds.setLEDMode(LEDMode.HasGamepiece);
        //         _timer.delay(0.005);
        //         _timer.stop();
        // break;
            // }
        // }
  
    }
        

    

    @Override
    public void end(boolean interrupted) {
        
    }
 
    @Override
    public boolean isFinished() {
        return true;
    }
}
