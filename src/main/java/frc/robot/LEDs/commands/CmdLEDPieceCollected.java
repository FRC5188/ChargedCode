package frc.robot.LEDs.commands;

// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDs.LEDs;
// import frc.robot.LEDs.LEDs.LEDMode;
import frc.robot.arm.Arm;
// import frc.robot.arm.Arm.ArmMode;
// import frc.robot.arm.Arm.ArmPosition;

public class CmdLEDPieceCollected extends CommandBase {
    //private Arm _armSubsystem;
    private LEDs _leds;
    // private DriverStation _driverStation;
    // private ArmMode armMode;
    //private ArmPosition armPosition;

    public CmdLEDPieceCollected(LEDs leds, Arm armSubsystem) {
        //_armSubsystem = armSubsystem;
        _leds = leds;
    }

    @Override
    public void initialize() {
        System.out.println("SETTING CANDLE LED TO TEAL FOR GAME PIECE");
        
    }

      @Override
    public void execute() {
        _leds.setRunningGamepieceAnimation(true);
  
    }
    
    @Override
    public void end(boolean interrupted) {}
 
    @Override
    public boolean isFinished() {
        return true;
    }
}
