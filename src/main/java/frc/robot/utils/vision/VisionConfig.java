// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConfig 
{
    public String cameraName;
    public Translation3d cameraPositionT;
    public Rotation3d cameraPositionR;
    public PoseStrategy primaryStrategy;
    public PoseStrategy fallBackStrategy;
    public boolean includeInPoseEstimates;

    @JsonCreator
    public VisionConfig(@JsonProperty(required = true, value="name") String name,
                        @JsonProperty(required = true, value="positionT") Translation3d camPositionT,
                        @JsonProperty(required = true, value="positionR") Rotation3d camPositionR,
                        @JsonProperty(required = true, value="primaryStrat") PoseStrategy primPoseStrategy,
                        @JsonProperty(required = true, value="backupStrat") PoseStrategy fallbackPoseStrategy,
                        @JsonProperty(required = false, value="includeInPoseEstimates", defaultValue="true") boolean includeInPoseEstimates) {
        this.cameraName = name;
        this.cameraPositionT = camPositionT;
        this.cameraPositionR = camPositionR;
        this.primaryStrategy = primPoseStrategy;
        this.fallBackStrategy = fallbackPoseStrategy;
        this.includeInPoseEstimates = includeInPoseEstimates;
    }
}