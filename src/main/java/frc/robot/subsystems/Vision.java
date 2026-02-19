// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.HashMap;
import java.util.Optional;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDBoolean;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.Configuration;
import frc.robot.utils.vision.VisionConfig;
import frc.robot.utils.vision.VisionEstimationResult;
import frc.robot.utils.vision.VisionSystem;

public class Vision extends SubsystemBase {

  private static Vision m_vision;
  private HashMap<String, VisionSystem> m_visionSystems;

  private TDNumber m_estX;
  private TDNumber m_estY;
  private TDNumber m_estRot;
  private TDBoolean m_poseUpdatesEnabled;
  private Field2d m_field;

  /** Creates a new Vision. */
  private Vision() {
    super("Vision");
    m_visionSystems = new HashMap<String, VisionSystem>();
    List<VisionConfig> myConfig = Configuration.getInstance().getVisionConfigs();
    if (myConfig != null)
    {
      // m_visionSystems.ensureCapacity(m_visionConfig.length);
      for(VisionConfig config : myConfig) {
         VisionSystem system = new VisionSystem(config);
         m_visionSystems.put(config.cameraName, system);
      }
    }
    m_estX = new TDNumber(this, "Est Pose", "Est X");
    m_estY = new TDNumber(this, "Est Pose", "Est Y");
    m_estRot = new TDNumber(this, "Est Pose", "Est Rot");
    m_field = new Field2d();
    //We don't care about the default robot object on this field, throw it into the abyss
    m_field.setRobotPose(-10, 0, Rotation2d.kZero);
    new TDSendable(this, "Field", "Vision Field", m_field);

    m_poseUpdatesEnabled = new TDBoolean(this, "", "Pose Updates Enabled", true);
  }

  public static Vision getInstance(){
    if(m_vision == null){
      m_vision = new Vision();
    }
    return m_vision;
  }

  public void enablePoseUpdates() {
    m_poseUpdatesEnabled.set(true);
  }

  public void disablePoseUpdates() {
    m_poseUpdatesEnabled.set(false);
  }

  public boolean getPoseUpdatesEnabled() {
    return m_poseUpdatesEnabled.get();
  }

  public Optional<VisionEstimationResult> getLatestFromCamera(String cameraName) {
    Optional<VisionEstimationResult> result = Optional.empty();
    if(m_visionSystems.containsKey(cameraName)) {
      var system = m_visionSystems.get(cameraName);
      result = system.getLatestEstimate();
    }
    return result;
  }

  @Override
  public void periodic() {
    if (RobotMap.V_ENABLED) {
      if(getPoseUpdatesEnabled()){
        Drive robotDrive = Drive.getInstance();

        for(var entry : m_visionSystems.entrySet()) {
          var system = entry.getValue();
          var newest = system.updateAndGetEstimatedPose();
          if(system.shouldIncludeInPoseEstimates()){
            newest.ifPresent(
              est -> {
                Pose2d estPose = est.estimatedPose.toPose2d();

                robotDrive.addVisionMeasurement(estPose, est.timestamp, est.stdDevs);

                m_field.getObject(system.getName()).setPose(estPose);
                m_estX.set(estPose.getX());
                m_estY.set(estPose.getY());
                m_estRot.set(estPose.getRotation().getDegrees());
              }
            );
          }
        }
      }
      super.periodic();
    }
  }
}