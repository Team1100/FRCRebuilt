package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.SubsystemBase;

public class Intake extends SubsystemBase{

    private static Intake m_Intake = null;

    private Intake(){
        super("Intake");
        if (RobotMap.I_ENABLED == true){
            
        }
    }

    public static Intake getInstance() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }
    
}
