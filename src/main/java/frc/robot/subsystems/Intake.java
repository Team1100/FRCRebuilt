package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.Configuration;
import frc.robot.utils.sensing.SparkCurrentLimitDetector;
import frc.robot.utils.sensing.SparkCurrentLimitDetector.HardLimitDirection;
import frc.robot.testingdashboard.SubsystemBase;


public class Intake extends SubsystemBase {
    private static Intake m_Intake = null;

	private boolean m_rollerEnabled;

    SparkBase m_rollerMotor;
    SparkBaseConfig m_rollerTopConfig;

	private boolean m_deployerEnabled;
    
    SparkBase m_deployMotor1;
    SparkBase m_deployMotor2;

    SparkBaseConfig m_deployConfig1;
    SparkBaseConfig m_deployConfig2;

    SparkCurrentLimitDetector m_deployLimitDetector;

    boolean m_deploying = false;
    double m_currentSpeed;

    TDNumber m_TDrollerBottomCurrentOutput;
    TDNumber m_TDrollerCurrentOutput;
    TDNumber m_TDdeployerCurrentOutput;

    private Intake(){
        super("Intake");

		m_rollerEnabled = cfgBool("rollerEnabled");
		m_deployerEnabled = cfgBool("deployerEnabled");

		if (m_rollerEnabled) {
      		var rollerTopMotorConfig = config().getMotorController("intakeRollerTop");
            m_rollerMotor = rollerTopMotorConfig.m_controller;

			m_rollerTopConfig = rollerTopMotorConfig.m_config;
            m_rollerTopConfig
                .idleMode(IdleMode.kCoast);
                //.smartCurrentLimit(cfgInt("rollerStallLimit"), cfgInt("rollerFreeLimit"));
            m_rollerMotor.configure(m_rollerTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_TDrollerCurrentOutput = new TDNumber(this, "Roller", "Measured Current");
		}

		if (m_deployerEnabled) {
            var deployConfig1 = config().getMotorController("intakeDeploy1");
            var deployConfig2 = config().getMotorController("intakeDeploy2");

            m_deployMotor1 = deployConfig1.m_controller;
            m_deployMotor2 = deployConfig2.m_controller;

            m_deployConfig1 = deployConfig1.m_config;
            m_deployConfig1
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(cfgInt("deployerStallLimit"), cfgInt("deployerFreeLimit"));

            m_deployConfig2 = deployConfig2.m_config;
            m_deployConfig2.idleMode(IdleMode.kBrake);
            m_deployConfig2.follow(m_deployMotor1, cfgBool("deployer2Invert"));

            m_deployMotor1.configure(m_deployConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_deployMotor2.configure(m_deployConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_deployLimitDetector = new SparkCurrentLimitDetector(m_deployMotor1, cfgDbl("deployerCurrentLimit"), cfgDbl("deployerZeroVelocityThreshold"));

            m_TDdeployerCurrentOutput = new TDNumber(this, "Intake", "Deployer Measured Current");
        }
    }


    public static Intake getInstance() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }

    public void spinIn(double speed) {
        if (m_rollerMotor != null) {
            m_rollerMotor.set(-speed);
        }
    }

    public void spinOut(double speed) {
        if (m_rollerMotor != null) {
            m_rollerMotor.set(speed);
        }
    }

    public void stop() {
        if (m_rollerMotor != null) {
            m_rollerMotor.set(0);
        }
    }

    public void deploy(double speed) {
        m_deploying = false;
        m_currentSpeed = speed;
    }

    public void retract(double speed) {
        m_deploying = true;
        m_currentSpeed = speed;
    }

	private void runRoller() {
		m_TDrollerCurrentOutput.set(m_rollerMotor.getOutputCurrent());
	}

	private void runDeployer() {
        if (m_deploying) {
            if (m_deployLimitDetector.check() != HardLimitDirection.kForward){
                m_deployMotor1.set(m_currentSpeed);
            } else {
                m_deployMotor1.set(0);
                m_currentSpeed = 0;
            }
        } else {
            if (m_deployLimitDetector.check() != HardLimitDirection.kReverse){
                m_deployMotor1.set(-m_currentSpeed);
            } else {
                m_deployMotor1.set(0);
                m_currentSpeed = 0;
            }
        }
        m_TDdeployerCurrentOutput.set(m_deployMotor1.getOutputCurrent());
	}
    
    @Override
    public void periodic() {
        if (m_rollerEnabled) runRoller();
		if (m_deployerEnabled) runDeployer();

        super.periodic();
    }
    
}
