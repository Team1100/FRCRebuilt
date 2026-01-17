package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.SubsystemBase;

public class Shooter extends SubsystemBase{

    private static Shooter m_Shooter = null;

    private Shooter(){
        super("Shooter");
        if (RobotMap.S_ENABLED == true){
            
        }

        
    }
    public static Shooter getInstance() {
        if (m_Shooter == null) {
            m_Shooter = new Shooter();
        }
        return m_Shooter;
    }
    
}
