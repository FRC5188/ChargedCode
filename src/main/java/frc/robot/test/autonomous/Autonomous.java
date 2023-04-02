package frc.robot.test.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.test.console_output.Output;

@Deprecated
public abstract class Autonomous {
    // Runs all autonomous routines sequentially. 
    public static Command runChecks(){
        return new InstantCommand(
            () -> {Output.error("Autonomous Not Implemented. No Tests Run.");}
        );
    }
}
