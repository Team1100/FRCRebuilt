package frc.robot.subsystems;

import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.utils.FieldUtils;
import edu.wpi.first.wpilibj.DriverStation;

public class LED extends SubsystemBase{

    private static LED m_LED = null;

    private LED(){
        super("LED");
    }

    public static LED getInstance() {
        if (m_LED == null) {
            m_LED = new LED();
        }
        return m_LED;
    }
    
    public void setLights(){
        char gameState = FieldUtils.getInstance().getGameState();
        double currentMatchTime = FieldUtils.getInstance().stateTimeLeft();
        if (currentMatchTime < cfgDbl("stateChangeWarningTime")){
            //something to alter the existing colors
        } 
        switch (gameState){

            case 'A': //auto

                break;
            case 'T' : //transition
                //can we make it trans colors :3?
                break;
            case 'R' : //red team active
                
                break;
            case 'B' : //blue team active
                
                break;
            case 'E' : //endgame
                
                break;
            default : 

                break;

        }
    }
    

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
    

}
