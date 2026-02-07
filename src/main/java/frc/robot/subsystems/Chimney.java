package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class Chimney extends SubsystemBase{

    SparkMax m_CSparkMax;
    SparkMaxConfig m_SparkMaxConfig;

    TDNumber td_currentOutput;

    private static Chimney m_Chimney = null;

    private Chimney(){
        super("Chimney");
        if (cfgBool("chimneyEnabled") == true){
            m_CSparkMax = new SparkMax(cfgInt("chimneyMotorCANid"), null);
            m_SparkMaxConfig = new SparkMaxConfig();
            m_SparkMaxConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("chimneyRollerStallLimit"), cfgInt("chimneyRollerFreeLimit"));

            m_CSparkMax.configure(m_SparkMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            td_currentOutput = new TDNumber(this, "Chimney", "Chimney");
                
        }
    }
    public static Chimney getInstance() {
        if (m_Chimney == null) {
            m_Chimney = new Chimney();
        }
        return m_Chimney;
    }
    public void spinIn(double speed) {
        if (m_CSparkMax != null) {
            m_CSparkMax.set(speed);
        }
    }
    public void spinOut(double speed) {
        if (m_CSparkMax != null) {
            m_CSparkMax.set(-1 * speed);
        }
    }
    public void stop() {
        if (m_CSparkMax != null) {
            m_CSparkMax.set(-0);
        }
    }
    public void santa(){
        System.out.println("ho ho ho, merry christmas");
    }
    
    @Override
    public void periodic() {
        if (cfgBool("chimneyEnabled") == true) {
            td_currentOutput.set(m_CSparkMax.getOutputCurrent());
        }
        super.periodic();
    }
    
}
