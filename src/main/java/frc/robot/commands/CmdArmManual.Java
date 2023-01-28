import frc.robot.subsystems.Arm;

public class CmdArmManual extends CommandBase {
    Arm





    Arm m_arm;
    DoubleSupplier m_upperJointDouble;
    DoubleSupplier m_lowerJointDouble;
    public CmdArmManual(Arm system, DoubleSupplier upperJointDouble, DoubleSupplier lowerJointDouble){
        m_arm = system;
        m_upperJointDouble = upperJointDouble;
        m_lowerJointDouble = lowerJointDouble;
    }
    public void execute(){

    }
    public boolean end(){
        return false;
    }
}