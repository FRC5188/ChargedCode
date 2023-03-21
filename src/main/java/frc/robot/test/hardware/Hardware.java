package frc.robot.test.hardware;

import edu.wpi.first.wpilibj2.command.Command;

@Deprecated
public abstract class Hardware {
    public static Command runRoboRIOChecks(){
        return null;
    }
    /// Checks if the battery voltage is sufficet. (12-13V)
    public static Command runBatteryChecks(){
        return null;
    }
}
