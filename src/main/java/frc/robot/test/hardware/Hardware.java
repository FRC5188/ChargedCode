package frc.robot.test.hardware;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.test.console_output.Output;

@Deprecated
public abstract class Hardware {

    public static Command check(){
        return null;       
    }

    private static boolean isCANBusConnected(){return RobotController.getCANStatus().percentBusUtilization <= 0;}

    private static boolean isBatteryCharged(){return (RobotController.getBatteryVoltage() < 10) ? (true): (false);}

}
