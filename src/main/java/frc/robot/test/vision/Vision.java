package frc.robot.test.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.test.console_output.Output;

@Deprecated
public abstract class Vision {

    public static Command runChecks(frc.robot.vision.Vision subsystem){
        return new InstantCommand(
            () -> {
                if(isServerConnected()){Output.sucess("Server is Connected");}
                else {Output.warning("Server Not Connected");}

                if(isTargetFound()){Output.sucess("Target Has Been Found");}
                else {Output.warning("Target Not Found");}
            }
        );
    }

    private static boolean isTargetFound(){
        return frc.robot.vision.Vision.hasTarget();
    }

    private static boolean isServerConnected(){
        return frc.robot.vision.Vision.isServerConnected();
    }

}
