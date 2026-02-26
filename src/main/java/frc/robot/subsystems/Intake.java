package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.Configuration;
import frc.robot.utils.sensing.SparkCurrentLimitDetector;
import frc.robot.utils.sensing.SparkCurrentLimitDetector.HardLimitDirection;
import frc.robot.testingdashboard.SubsystemBase;


public class Intake extends SubsystemBase{
    SparkBase m_rollerBottomMotor;
    SparkBase m_rollerTopMotor;
    SparkBase m_deployMotor1;
    SparkBase m_deployMotor2;
    SparkBaseConfig m_rollerBottomConfig;
    SparkBaseConfig m_rollerTopConfig;
    SparkBaseConfig m_deployerConfig1;
    SparkBaseConfig m_deployerConfig2;
    SparkCurrentLimitDetector sparkCurrentLimitDetector;

    boolean m_deploying = false;
    double m_currentSpeed;

    TDNumber m_TDrollerBottomCurrentOutput;
    TDNumber m_TDrollerTopCurrentOutput;
    TDNumber m_TDdeployerCurrentOutput;

	TDNumber m_TDrollerSpeed;

	private boolean m_rollerEnabled;
	private boolean m_deployerEnabled;

    private static Intake m_Intake = null;

    private Intake(){
        super("Intake");

		m_rollerEnabled = cfgBool("rollerEnabled");
		m_deployerEnabled = cfgBool("deployerEnabled");

		if (m_rollerEnabled) {
      		//var rollerBottomMotorConfig = config().getMotorController("intakeRollerBottom");
      		var rollerTopMotorConfig = config().getMotorController("intakeRollerTop");
            //m_rollerBottomMotor = rollerBottomMotorConfig.m_controller;
            m_rollerTopMotor = rollerTopMotorConfig.m_controller;

            /*m_rollerBottomConfig = rollerBottomMotorConfig.m_config;
            m_rollerBottomConfig
                .idleMode(IdleMode.kCoast);
                //.smartCurrentLimit(cfgInt("rollerStallLimit"), cfgInt("rollerFreeLimit"));
            m_rollerBottomMotor.configure(m_rollerBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);*/

			m_rollerTopConfig = rollerTopMotorConfig.m_config;
            m_rollerTopConfig
                .idleMode(IdleMode.kCoast);
                //.smartCurrentLimit(cfgInt("rollerStallLimit"), cfgInt("rollerFreeLimit"));
            m_rollerTopMotor.configure(m_rollerTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

			m_TDrollerSpeed = new TDNumber(this, "Roller", "Speed");

            m_TDrollerBottomCurrentOutput = new TDNumber(this, "Roller", "Bottom Measured Current");
            m_TDrollerTopCurrentOutput = new TDNumber(this, "Roller", "Top Measured Current");
		}

		if (m_deployerEnabled) {
            m_deployMotor1 = new SparkFlex(cfgInt("deployerCanId1"), MotorType.kBrushless);
            m_deployMotor2 = new SparkFlex(cfgInt("deployerCanId2"), MotorType.kBrushless);

            m_deployerConfig1 = new SparkFlexConfig();
            m_deployerConfig1
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("deployerStallLimit"), cfgInt("deployerFreeLimit"));

            m_deployerConfig2 = new SparkFlexConfig();
            m_deployerConfig2
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("intakeDeployerStallLimit"), cfgInt("intakeDeployerFreeLimit"));
            m_deployerConfig2.follow(cfgInt("intakeDeployerCanId1"), cfgBool("intakeDeployer2Invert"));

            m_deployMotor1.configure(m_deployerConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_deployMotor2.configure(m_deployerConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_TDdeployerCurrentOutput = new TDNumber(this, "Intake", "Deployer Measured Current");
        }
    }


    public static Intake getInstance() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }

    //ROLLER METHODS
    public void spinIn(double speed) {
        if (m_rollerBottomMotor != null) {
            //m_rollerBottomMotor.set(speed*m_TDrollerSpeed.get());
        }
        if (m_rollerTopMotor != null) {
            m_rollerTopMotor.set(speed*m_TDrollerSpeed.get());
        }
    }
    public void spinOut(double speed) {
        if (m_rollerBottomMotor != null) {
            //m_rollerBottomMotor.set(-speed*m_TDrollerSpeed.get());
        }
        if (m_rollerTopMotor != null) {
            m_rollerTopMotor.set(-speed*m_TDrollerSpeed.get());
        }
    }
    public void stop() {
        if (m_rollerBottomMotor != null) {
            //m_rollerBottomMotor.set(0);
        }
        if (m_rollerTopMotor != null) {
            m_rollerTopMotor.set(0);
        }
    }

    //DEPLOYER METHODS
    public void deploy(double speed) {
        m_deploying = true;
        m_currentSpeed = speed;
    }

    public void retract(double speed) {
        m_deploying = false;
        m_currentSpeed = speed;
    }

	private void runRoller() {
		//m_TDrollerBottomCurrentOutput.set(m_rollerBottomMotor.getOutputCurrent());
		m_TDrollerTopCurrentOutput.set(m_rollerTopMotor.getOutputCurrent());
	}

	private void runDeployer() {
        if (m_deploying) {
            if (sparkCurrentLimitDetector.check() != HardLimitDirection.kForward){
                m_deployMotor1.set(m_currentSpeed);
            } else {
                m_deployMotor1.set(0);
            }
        } else {
            if (sparkCurrentLimitDetector.check() != HardLimitDirection.kReverse){
                m_deployMotor1.set(-m_currentSpeed);
            } else {
                m_deployMotor1.set(0);
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
