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

    SparkFlex m_IRSparkFlex;
    SparkFlex m_ID1SparkFlex;
    SparkFlex m_ID2SparkFlex;
    SparkFlexConfig m_RSparkFlexConfig;
    SparkFlexConfig m_D1SparkFlexConfig;
    SparkFlexConfig m_D2SparkFlexConfig;
    SparkCurrentLimitDetector sparkCurrentLimitDetector;

    boolean deploying = false;
    double currentSpeed;

    TDNumber td_currentOutput;

    private static Intake m_Intake = null;

    private Intake(){
        super("Intake");
        if (cfgBool("intakeEnabled") == true){
            m_IRSparkFlex = new SparkFlex(cfgInt("intakeRollerCanId"), null);
            m_ID1SparkFlex = new SparkFlex(cfgInt("intakeDeployerCanId1"), null);
            m_ID2SparkFlex = new SparkFlex(cfgInt("intakeDeployerCanId2"), null);
            m_RSparkFlexConfig = new SparkFlexConfig();
            m_RSparkFlexConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("intakeRollerStallLimit"), cfgInt("intakeRollerFreeLimit"));
            m_D1SparkFlexConfig = new SparkFlexConfig();
            m_D1SparkFlexConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("intakeDeployerStallLimit"), cfgInt("intakeDeployerFreeLimit"));

            m_D2SparkFlexConfig = new SparkFlexConfig();
            m_D2SparkFlexConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(cfgInt("intakeDeployerStallLimit"), cfgInt("intakeDeployerFreeLimit"));
            m_D2SparkFlexConfig.follow(cfgInt("intakeDeployerCanId1"), cfgBool("intakeDeployer2Invert"));

            m_IRSparkFlex.configure(m_RSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_ID1SparkFlex.configure(m_D1SparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_ID2SparkFlex.configure(m_D2SparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            td_currentOutput = new TDNumber(this, "Intake", "Ground Intake");
                
        }
    }

    //double apple = Configuration.getInstance().getDouble("Drive", "testDouble");
    //double fhuwi = cfgDbl("testDouble"); //how to get variable from config 

    public static Intake getInstance() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }
    //ROLLER METHODS
    public void spinIn(double speed) {
        if (m_IRSparkFlex != null) {
            m_IRSparkFlex.set(speed);
        }
    }
    public void spinOut(double speed) {
        if (m_IRSparkFlex != null) {
            m_IRSparkFlex.set(-1 * speed);
        }
    }
    public void stop() {
        if (m_IRSparkFlex != null) {
            m_IRSparkFlex.set(-0);
        }
    }
    //DEPLOYER METHODS
    public void deploy(double speed) {
        deploying = true;
        currentSpeed = speed;
    }

    public void retract(double speed) {
        deploying = false;
        currentSpeed = speed;
    }
    
    @Override
    public void periodic() {
        if (deploying == true){
            if (sparkCurrentLimitDetector.check() != HardLimitDirection.kForward){
                m_ID1SparkFlex.set(currentSpeed);
            } else {
                m_ID1SparkFlex.set(-0);
            }
        } else if (deploying == false){
            if (sparkCurrentLimitDetector.check() != HardLimitDirection.kReverse){
                m_ID1SparkFlex.set(-1 * currentSpeed);
            } else {
                m_ID1SparkFlex.set(-0);
            }
        }

        if (cfgBool("intakeEnabled") == true) {
            td_currentOutput.set(m_IRSparkFlex.getOutputCurrent());
            td_currentOutput.set(m_ID1SparkFlex.getOutputCurrent());
            td_currentOutput.set(m_ID2SparkFlex.getOutputCurrent());
        }
        super.periodic();
    }
    
}
