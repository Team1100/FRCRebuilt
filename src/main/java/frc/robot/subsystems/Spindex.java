package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class Spindex extends SubsystemBase{

    SparkMax m_SSparkMax;
    SparkMaxConfig m_SparkMaxConfig;

    TDNumber td_currentOutput;

    private static Spindex m_Spindex = null;
    
    private Spindex(){
        super("Spindex");
        if (cfgBool("spindexEnabled") == true){
            m_SSparkMax = new SparkMax(cfgInt("spindexCanId"), null);
            m_SparkMaxConfig = new SparkMaxConfig();
            m_SparkMaxConfig
                .idleMode(null)
                .smartCurrentLimit(cfgInt("spindexStallLimit"), cfgInt("spindexFreeLimit"));
            
            m_SSparkMax.configure(m_SparkMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            td_currentOutput = new TDNumber(this, "Spindex", "Spindex");
        }
        
    }


    public static Spindex getInstance() {
        if (m_Spindex == null) {
            m_Spindex = new Spindex();
        }
        return m_Spindex;
    }
    public void spinIn(double speed) {
        if (m_SSparkMax != null) {
            m_SSparkMax.set(speed);
        }
    }
    public void spinOut(double speed) {
        if (m_SSparkMax != null) {
            m_SSparkMax.set(-1 * speed);
        }
    }
    public void stop() {
        if (m_SSparkMax != null) {
            m_SSparkMax.set(-0);
        }
    }
    
    @Override
    public void periodic() {
        if (cfgBool("spindexEnabled") == true) {
            td_currentOutput.set(m_SSparkMax.getOutputCurrent());
        }
        super.periodic();
    }
}
