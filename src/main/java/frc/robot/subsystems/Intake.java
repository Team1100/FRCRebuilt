package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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

    SparkFlex m_rollerMotor;
    SparkFlex m_deployMotor1;
    SparkFlex m_deployMotor2;
    SparkFlexConfig m_rollerConfig;
    SparkFlexConfig m_deployerConfig1;
    SparkFlexConfig m_deployerConfig2;
    SparkCurrentLimitDetector sparkCurrentLimitDetector;

    boolean m_deploying = false;
    double m_currentSpeed;

    TDNumber m_TDrollerCurrentOutput;
    TDNumber m_TDdeployerCurrentOutput;

    private static Intake m_Intake = null;

    private Intake(){
        super("Intake");
        if (cfgBool("intakeEnabled") == true){
            m_rollerMotor = new SparkFlex(cfgInt("intakeRollerCanId"), MotorType.kBrushless);
            m_deployMotor1 = new SparkFlex(cfgInt("intakeDeployerCanId1"), MotorType.kBrushless);
            m_deployMotor2 = new SparkFlex(cfgInt("intakeDeployerCanId2"), MotorType.kBrushless);
            m_rollerConfig = new SparkFlexConfig();
            m_rollerConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("intakeRollerStallLimit"), cfgInt("intakeRollerFreeLimit"));
            m_deployerConfig1 = new SparkFlexConfig();
            m_deployerConfig1
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("intakeDeployerStallLimit"), cfgInt("intakeDeployerFreeLimit"));

            m_deployerConfig2 = new SparkFlexConfig();
            m_deployerConfig2
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("intakeDeployerStallLimit"), cfgInt("intakeDeployerFreeLimit"));
            m_deployerConfig2.follow(cfgInt("intakeDeployerCanId1"), cfgBool("intakeDeployer2Invert"));

            m_rollerMotor.configure(m_rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_deployMotor1.configure(m_deployerConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_deployMotor2.configure(m_deployerConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_TDrollerCurrentOutput = new TDNumber(this, "Intake", "Roller Measured Current");
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
        if (m_rollerMotor != null) {
            m_rollerMotor.set(speed);
        }
    }
    public void spinOut(double speed) {
        if (m_rollerMotor != null) {
            m_rollerMotor.set(-1 * speed);
        }
    }
    public void stop() {
        if (m_rollerMotor != null) {
            m_rollerMotor.set(-0);
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
    
    @Override
    public void periodic() {
        if (m_deploying){
            if (sparkCurrentLimitDetector.check() != HardLimitDirection.kForward){
                m_deployMotor1.set(m_currentSpeed);
            } else {
                m_deployMotor1.set(-0);
            }
        } else {
            if (sparkCurrentLimitDetector.check() != HardLimitDirection.kReverse){
                m_deployMotor1.set(-1 * m_currentSpeed);
            } else {
                m_deployMotor1.set(-0);
            }
        }

        if (cfgBool("intakeEnabled") == true) {
            m_TDrollerCurrentOutput.set(m_rollerMotor.getOutputCurrent());
            m_TDdeployerCurrentOutput.set(m_deployMotor1.getOutputCurrent());
        }
        super.periodic();
    }
    
}
