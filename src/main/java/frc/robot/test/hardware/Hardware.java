package frc.robot.test.hardware;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.test.console_output.Output;

@Deprecated
public abstract class Hardware {
    /**
    <Strong>Test One: </Strong>Is the CAN Bus connected? <p>
    <Strong>Test Two: </Strong>Is the Battery Charged?
    **/
    public static Command check(){
        return new InstantCommand(
        () -> {
        if(isCANBusConnected()){Output.sucess("CAN Bus Is Connected");}
        else {Output.error("CAN Bus Isn't Connected. Zero Utilization");}

        if(isBatteryCharged()){Output.sucess("Battery Charged");}
        else {Output.warning("Battery Isn't Fully Charged");}
        });
    }

    private static boolean isCANBusConnected(){return RobotController.getCANStatus().percentBusUtilization <= 0;}

    private static boolean isBatteryCharged(){return (RobotController.getBatteryVoltage() < 10) ? (true): (false);}

}
