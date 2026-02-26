// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDBoolean;
import frc.robot.testingdashboard.TDNumber;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private static Climber m_Climber;

  TDNumber m_targetAngle;
  TDNumber m_climberEncoderValueInches;
  TDNumber m_climberCurrentOutput;
  TDNumber m_TDclimberP;
  TDNumber m_TDclimberI;
  TDNumber m_TDclimberD;
  TDNumber m_TDclimberKs;
  TDNumber m_TDclimberKg;
  TDNumber m_TDclimberKv;
  TDNumber m_TDclimberKa;
  TDNumber m_TDclimberFFout;
  double m_climberP = Constants.ClimberConstants.kClimberP;
  double m_climberI = Constants.ClimberConstants.kClimberI;
  double m_climberD = Constants.ClimberConstants.kClimberD;
  double m_climberkS = Constants.ClimberConstants.kClimberkS;
  double m_climberkG = Constants.ClimberConstants.kClimberkG;
  double m_climberkV = Constants.ClimberConstants.kClimberkV;
  double m_climberkA = Constants.ClimberConstants.kClimberkA;
  private double m_climberLastAngle = 0;

  SparkBase m_climberLeftSpark;
  SparkBase m_climberRightSpark;

  TDBoolean m_TDHighLimitHit;
  TDBoolean m_TDLowLimitHit;

  TDNumber m_climberLeftCurrentOutput;
  TDNumber m_climberRightCurrentOutput;

  SparkClosedLoopController m_climberClosedLoopController;
  RelativeEncoder m_climberMotorEncoder;

  // Making Trapezoid Profile & FeedForoward
  ElevatorFeedforward m_climberFeedForwardController;
  TrapezoidProfile m_climberProfile;
  TrapezoidProfile.State m_climberState;
  TrapezoidProfile.State m_climberSetpoint;
  TDNumber m_TDclimberProfilePosition;

  SparkBaseConfig m_leftSparkConfig;

  private final DCMotor m_climberMotor = DCMotor.getNEO(2);

  // Cool Climber Simulator that I took from ReefScape Elevator
  private final ElevatorSim m_climberSim =
      new ElevatorSim(m_climberMotor,
       20,
       5.0,
       0.10,
       0.0,
       1.0,
       true,
       0.0);
  private Encoder m_encoder;
  private EncoderSim m_encoderSim;
  private DCMotorSim m_motorSim;

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Climber Root", 10, 0);
  private final MechanismLigament2d m_climberMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Climber", m_climberSim.getPositionMeters() * 20, 90));

  private Climber() {
    super("Climber");

    if (RobotMap.C_ENABLED) {
      // Setup Climber (Yes this is just Ctrl C & V)
      
      var sparkAndConfigRight = config().getMotorController("climberRight");
      var sparkAndConfigLeft  = config().getMotorController("climberLeft");
      m_climberLeftSpark = sparkAndConfigLeft.m_controller;
      m_climberRightSpark = sparkAndConfigRight.m_controller;

      m_leftSparkConfig = sparkAndConfigLeft.m_config;
      var rightClimberSparkConfig = sparkAndConfigRight.m_config;

      rightClimberSparkConfig.follow(m_climberLeftSpark, true);

      m_TDclimberP = new TDNumber(this, "Climber PID", "climbP", Constants.ClimberConstants.kClimberP);
      m_TDclimberI = new TDNumber(this, "Climber PID", "climbI", Constants.ClimberConstants.kClimberI);
      m_TDclimberD = new TDNumber(this, "Climber PID", "climbD", Constants.ClimberConstants.kClimberD);
      m_TDclimberKg = new TDNumber(this, "Climber PID", "climbkG", Constants.ClimberConstants.kClimberkG);
      m_TDclimberKs = new TDNumber(this, "Climber PID", "climbkS", Constants.ClimberConstants.kClimberkS);
      m_TDclimberKv = new TDNumber(this, "Climber PID", "climbkV", Constants.ClimberConstants.kClimberkV);
      m_TDclimberKa = new TDNumber(this, "Climber PID", "climbkA", Constants.ClimberConstants.kClimberkA);

      m_TDclimberFFout = new TDNumber(this, "Climber PID", "FF Out");

      m_leftSparkConfig.closedLoop.pid(Constants.ClimberConstants.kClimberP, Constants.ClimberConstants.kClimberI,
          Constants.ClimberConstants.kClimberD);
      m_leftSparkConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      m_leftSparkConfig.closedLoop.positionWrappingEnabled(false);

      m_climberClosedLoopController = m_climberLeftSpark.getClosedLoopController();
      m_climberMotorEncoder = m_climberLeftSpark.getEncoder();
      m_climberMotorEncoder.setPosition(0);

      m_climberFeedForwardController = new ElevatorFeedforward(Constants.ClimberConstants.kClimberkS, Constants.ClimberConstants.kClimberkG, Constants.ClimberConstants.kClimberkV, Constants.ClimberConstants.kClimberkA);
      m_climberProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        Constants.ClimberConstants.kClimberMaxVelocity,
        Constants.ClimberConstants.kClimberMaxAcceleration
      ));
      m_climberSetpoint = new TrapezoidProfile.State(m_climberMotorEncoder.getPosition(), 0.0);
      m_climberState = new TrapezoidProfile.State(m_climberMotorEncoder.getPosition(), 0.0);

      m_TDclimberProfilePosition = new TDNumber(this, "Climber PID", "Profile Position");

      m_climberEncoderValueInches = new TDNumber(this, "Climber Encoder Values", "Height (inches)", getClimberAngle());
      m_climberLeftCurrentOutput = new TDNumber(this, "Current", "Left Climber Output", m_climberLeftSpark.getOutputCurrent());
      m_climberRightCurrentOutput = new TDNumber(this, "Current", "Right Climber Output", m_climberRightSpark.getOutputCurrent());
    }
  }

  public static Climber getInstance() {
    if (m_Climber == null) {
      m_Climber = new Climber();
    }
    return m_Climber;
  }

  public void FunnyClimbAction() {
  }

  public void FunnyClimbAscendAction() {
  }

  public void FunnyClimbDescendAction() {
  }

  public void FunnyClimbHaltAction() {
  }

  public double getClimberTargetAngle() {
    return m_climberLastAngle;
  }

  public double getClimberAngle() {
    return m_climberMotorEncoder.getPosition();
  }

  public boolean isClimberOnRung() {
    return false;
  }

  public void setClimberTargetAngle(double positionInches) {
    positionInches = MathUtil.clamp(positionInches,
                              Constants.ClimberConstants.kClimberLowerLimitInches, 
                              Constants.ClimberConstants.kClimberUpperLimitInches);
    if (positionInches != m_climberLastAngle) {
      m_targetAngle.set(positionInches);
      m_climberLastAngle = positionInches;
      m_climberSetpoint = new TrapezoidProfile.State(positionInches, 0.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.ClimberConstants.kEnableClimberPIDTuning &&
        m_climberLeftSpark != null) {
      double tmp = m_TDclimberP.get();
      boolean changed = false;
      if (tmp != m_climberP) {
        m_climberP = tmp;
        m_leftSparkConfig.closedLoop.p(m_climberP);
        changed = true;
      }
      tmp = m_TDclimberI.get();
      if (tmp != m_climberI) {
        m_climberI = tmp;
        changed = true;
        m_leftSparkConfig.closedLoop.i(m_climberI);
      }
      tmp = m_TDclimberD.get();
      if (tmp != m_climberD) {
        m_climberD = tmp;
        changed = true;
        m_leftSparkConfig.closedLoop.d(m_climberD);
      }
      if(changed) {
        m_climberLeftSpark.configure(m_leftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

      boolean ffchanged = false;
      tmp = m_TDclimberKg.get();
      if(tmp != m_climberkG) {
        m_climberkG = tmp;
        ffchanged = true;
      }
      tmp = m_TDclimberKv.get();
      if(tmp != m_climberkV) {
        m_climberkV = tmp;
        ffchanged = true;
      }
      tmp = m_TDclimberKs.get();
      if(tmp != m_climberkS)
      {
        m_climberkS = tmp;
        ffchanged = true;
      }
      tmp = m_TDclimberKa.get();
      if(tmp != m_climberkA)
      {
        m_climberkA = tmp;
        ffchanged = true;
      }
      if(ffchanged) {
        m_climberFeedForwardController = new ElevatorFeedforward(m_climberkS, m_climberkG, m_climberkV, m_climberkA);
      }
      tmp = m_targetAngle.get();
      if(tmp != m_climberLastAngle)
      {
        setClimberTargetAngle(tmp);
      }
    }

    m_climberState = m_climberProfile.calculate(Constants.schedulerPeriodTime, m_climberState, m_climberSetpoint);

    double climberFeedForward = m_climberFeedForwardController.calculate(m_climberState.velocity);

    m_climberClosedLoopController.setSetpoint(m_climberState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, climberFeedForward, ArbFFUnits.kVoltage);
  }
}
