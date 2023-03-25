package frc.robot.test.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.test.console_output.Output;

@Deprecated
public abstract class Vision {
    private static frc.robot.vision.Vision subsystem;

    public static Command runChecks(frc.robot.vision.Vision subsystem){
        Vision.subsystem = subsystem;

        return new InstantCommand(
            () -> {
                if(isServerConnected()){Output.sucess("Server is Connected");}
                else {Output.warning("Server Not Connected");}

                if(isCameraConnected()){Output.sucess("Camera is Connected");}
                else {Output.warning("Camera is Connected");}

                if(isTargetFound()){Output.sucess("Target Has Been Found");}
                else {Output.warning("Target Not Found");}

                Output.information(getTargetInformation());
            }
        );
    }

    private static boolean isTargetFound(){
        return false;
    }

    private static boolean isServerConnected(){
        return false;
    }

    private static boolean isCameraConnected(){
        return false;
    }

    private static String getTargetInformation(){
        return "";
    }
}
