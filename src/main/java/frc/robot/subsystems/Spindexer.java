package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class Spindexer extends SubsystemBase {
    SparkBase m_spindexerMotor;
    SparkBaseConfig m_spindexerConfig;

    TDNumber td_currentOutput;

	private boolean m_spindexerEnabled;

    private static Spindexer m_Spindexer = null;
    
    private Spindexer() {
        super("Spindexer");
		m_spindexerEnabled = cfgBool("spindexerEnabled");
        if (m_spindexerEnabled) {
			var spindexerMotorConfig = config().getMotorController("spindexer");
            m_spindexerMotor = spindexerMotorConfig.m_controller;
            m_spindexerConfig = spindexerMotorConfig.m_config;
            m_spindexerConfig
                .idleMode(IdleMode.kCoast)
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
            m_spindexerMotor.set(-speed);
        }
    }

    public void spinOut(double speed) {
        if (m_spindexerMotor != null) {
            m_spindexerMotor.set(speed);
        }
    }

    public void stop() {
        if (m_spindexerMotor != null) {
            m_spindexerMotor.set(0);
        }
    }
    
    @Override
    public void periodic() {
        if (m_spindexerEnabled) td_currentOutput.set(m_spindexerMotor.getOutputCurrent());

        super.periodic();
    }
}
