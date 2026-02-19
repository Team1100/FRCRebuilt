package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class Spindexer extends SubsystemBase {
    SparkMax m_spindexerMotor;
    SparkMaxConfig m_spindexerConfig;

    TDNumber td_currentOutput;

    private static Spindexer m_Spindexer = null;
    
    private Spindexer() {
        super("Spindexer");
        if (cfgBool("spindexerEnabled") == true) {
            m_spindexerMotor = new SparkMax(cfgInt("spindexerCanId"), null);
            m_spindexerConfig = new SparkMaxConfig();
            m_spindexerConfig
                .idleMode(null)
                .smartCurrentLimit(cfgInt("spindexerStallLimit"), cfgInt("spindexerFreeLimit"));
            
            m_spindexerMotor.configure(m_spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            td_currentOutput = new TDNumber(this, "Spindexer", "Measured Current");
        }
    }

    public static Spindexer getInstance() {
        if (m_Spindexer == null) {
            m_Spindexer = new Spindexer();
        }
        return m_Spindexer;
    }

    public void spinIn(double speed) {
        if (m_spindexerMotor != null) {
            m_spindexerMotor.set(speed);
        }
    }
    public void spinOut(double speed) {
        if (m_spindexerMotor != null) {
            m_spindexerMotor.set(-1 * speed);
        }
    }
    public void stop() {
        if (m_spindexerMotor != null) {
            m_spindexerMotor.set(-0);
        }
    }
    
    @Override
    public void periodic() {
        if (cfgBool("spindexerEnabled") == true) {
            td_currentOutput.set(m_spindexerMotor.getOutputCurrent());
        }
        super.periodic();
    }
}
