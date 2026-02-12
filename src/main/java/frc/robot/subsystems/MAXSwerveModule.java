// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.Configuration;

public class MAXSwerveModule {
  private final SparkBase m_drivingSpark;
  private final SparkBase m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private double m_lastAngle = 0;
  private double m_lastSpeed = 0;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_position = Meters.mutable(0);
  private final MutLinearVelocity m_speed = MetersPerSecond.mutable(0);


  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(Configuration.ControllerAndConfig driving, 
                         Configuration.ControllerAndConfig turning,
                         double chassisAngularOffset) {
    m_drivingSpark = driving.m_controller; 
    m_turningSpark = turning.m_controller;

    /* Configure the PID, FF and other properties of the turning motors. Most of these come from 
     * Constants.
    */
    SparkBaseConfig m_turningConfig = turning.m_config;
    m_turningConfig.idleMode(SwerveModuleConstants.kTurningMotorIdleMode);
    m_turningConfig.smartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

    m_turningConfig.absoluteEncoder
        .inverted(SwerveModuleConstants.kTurningEncoderInverted)
        .positionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);

    m_turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(SwerveModuleConstants.kTurningP, SwerveModuleConstants.kTurningI, SwerveModuleConstants.kTurningD)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput)
        .positionWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput)
        .outputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput);

    m_turningSpark.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    m_turningPIDController = m_turningSpark.getClosedLoopController();

    /* Configure the PID, FFF and other properties of the driving motors. Most of these come from 
     * Constants.
    */
    SparkBaseConfig m_drivingConfig = driving.m_config;
    m_drivingConfig.idleMode(SwerveModuleConstants.kDrivingMotorIdleMode);
    m_drivingConfig.inverted(false);
    m_drivingConfig.smartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
    
    m_drivingConfig.encoder
        .positionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);

    m_drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(SwerveModuleConstants.kDrivingP, SwerveModuleConstants.kDrivingI, SwerveModuleConstants.kDrivingD)
        .outputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput)
        .feedForward.kV(SwerveModuleConstants.kDrivingVelocityFF);

    m_drivingSpark.configure(m_drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_drivingPIDController = m_drivingSpark.getClosedLoopController();
   
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    System.out.println("driving P: " + SwerveModuleConstants.kDrivingP +
                             ", I: " + SwerveModuleConstants.kDrivingI +
                             ", D: " + SwerveModuleConstants.kDrivingD);

    System.out.println("turning P: " + SwerveModuleConstants.kTurningP +
                             ", I: " + SwerveModuleConstants.kTurningI +
                             ", D: " + SwerveModuleConstants.kTurningD);
  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    
    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    if (correctedDesiredState.speedMetersPerSecond != m_lastSpeed) {
      m_lastSpeed = correctedDesiredState.speedMetersPerSecond;
      m_drivingPIDController.setSetpoint(correctedDesiredState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
    }
    if (correctedDesiredState.angle.getRadians() != m_lastAngle) {
      m_lastAngle = correctedDesiredState.angle.getRadians();
      m_turningPIDController.setSetpoint(correctedDesiredState.angle.getRadians(), SparkBase.ControlType.kPosition);
    }

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getLastSpeed()
  {
    return m_lastSpeed;
  }

  // sysid helpers
  public void goSysid(Voltage in) {
    m_drivingSpark.set(in.magnitude()/RobotController.getBatteryVoltage());
    m_turningPIDController.setSetpoint(new Rotation2d().plus(Rotation2d.fromRadians(m_chassisAngularOffset)).getRadians(),
                                       ControlType.kPosition);
  }

  public Voltage getDriveVoltage() {
    return m_appliedVoltage.mut_replace(m_drivingSpark.get() * RobotController.getBatteryVoltage(), Volts);
  }
  public Distance getDrivePosition() {
    return m_position.mut_replace(m_drivingEncoder.getPosition(), Meters);
  }
  public LinearVelocity getDriveVelocity() {
    return m_speed.mut_replace(m_drivingEncoder.getVelocity(), MetersPerSecond);
  }

  public double getDriveOutputCurrent() {
    return m_drivingSpark.getOutputCurrent();
  }
  public double getTurningOutputCurrent() {
    return m_turningSpark.getOutputCurrent();
  }
}