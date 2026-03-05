// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionConfig 
{
    public String cameraName;
    public Translation3d cameraTranslation;
    public Rotation3d cameraRotation;
    public PoseStrategy primaryStrategy;
    public PoseStrategy fallBackStrategy;
    public boolean includeInPoseEstimates;

    @JsonCreator
    public VisionConfig(@JsonProperty(required = true, value="name") String name,
                        @JsonProperty(required = true, value="translation") Translation3d camTranslation,
                        @JsonProperty(required = true, value="rotation") Rotation3d camRotation,
                        @JsonProperty(required = true, value="primaryStrat") PoseStrategy primPoseStrategy,
                        @JsonProperty(required = true, value="backupStrat") PoseStrategy fallbackPoseStrategy,
                        @JsonProperty(required = false, value="includeInPoseEstimates", defaultValue="true") boolean includeInPoseEstimates) {
        this.cameraName = name;
        this.cameraTranslation = camTranslation;
        this.cameraRotation = camRotation;
        this.primaryStrategy = primPoseStrategy;
        this.fallBackStrategy = fallbackPoseStrategy;
        this.includeInPoseEstimates = includeInPoseEstimates;
    }

    private static class EulerAngles {
        private double m_pitch;
        private double m_yaw;
        private double m_roll;

        @JsonCreator
        public EulerAngles(@JsonProperty(required = true, value="pitch") double pitch,
                           @JsonProperty(required = true, value="yaw") double yaw,
                           @JsonProperty(required = true, value="roll") double roll,
                           @JsonProperty(required = false, value="inRadians", defaultValue = "false") boolean inRadians) {
            m_pitch = pitch;
            m_yaw = yaw;
            m_roll = roll;

            if (!inRadians) {
                m_pitch *= Math.PI/180.0;
                m_yaw *= Math.PI/180.0;
                m_roll *= Math.PI/180.0;
            }
        }

        public Rotation3d getRotation() {
            return new Rotation3d(m_roll, m_pitch, m_yaw);
        }
    }
    @JsonCreator
    public VisionConfig(@JsonProperty(required = true, value="name") String name,
                        @JsonProperty(required = true, value="translation") Translation3d camTranslation,
                        @JsonProperty(required = true, value="angles") EulerAngles camAngles,
                        @JsonProperty(required = true, value="primaryStrat") PoseStrategy primPoseStrategy,
                        @JsonProperty(required = true, value="backupStrat") PoseStrategy fallbackPoseStrategy,
                        @JsonProperty(required = false, value="includeInPoseEstimates", defaultValue="true") boolean includeInPoseEstimates) {
        this.cameraName = name;
        this.cameraTranslation = camTranslation;
        this.cameraRotation = camAngles.getRotation();
        this.primaryStrategy = primPoseStrategy;
        this.fallBackStrategy = fallbackPoseStrategy;
        this.includeInPoseEstimates = includeInPoseEstimates;
    }

}