package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.utils.FieldUtils;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase{

    private static LED m_LEDLights = null;

    private AddressableLED m_LED;
    private AddressableLEDBuffer m_LEDBuffer;

    private LED(){
        super("LED");

         m_LED = new AddressableLED(cfgInt("LEDPort"));

        m_LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LEDLength);
        m_LED.setLength(Constants.LEDConstants.LEDLength);

        m_LED.start(); //public static int L_LEDS = RoboRioMap.PWM_0;

    }

    public static LED getInstance() {
        if (m_LEDLights == null) {
            m_LEDLights = new LED();
        }
        return m_LEDLights;
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
