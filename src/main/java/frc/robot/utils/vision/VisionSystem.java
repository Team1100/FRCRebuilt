// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive;;

/** Add your docs here. */
public class VisionSystem {

    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_photonEstimator;
    private double m_lastEstTime;
    private Optional<VisionEstimationResult> m_latestResult;
    private PoseStrategy m_multiStrategy;
    private PoseStrategy m_singleStrategy;
    private boolean m_includeInPoseEstimates;

    public VisionSystem(VisionConfig config) {
        m_camera = new PhotonCamera(config.cameraName);
        m_photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, new Transform3d(config.cameraPositionT, config.cameraPositionR));
        if((config.primaryStrategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) ||
            (config.primaryStrategy == PoseStrategy.MULTI_TAG_PNP_ON_RIO)) {
            m_multiStrategy = config.primaryStrategy;
            m_singleStrategy = config.fallBackStrategy;
        } else {
            m_singleStrategy = config.primaryStrategy;
        }
        m_latestResult = Optional.empty();
        m_includeInPoseEstimates = config.includeInPoseEstimates;
    }

    public String getName() { return m_camera.getName(); }

    public Optional<VisionEstimationResult> getLatestEstimate() {
        return m_latestResult;
    }

    public PhotonPipelineResult getLatestResult() {
        if(m_camera != null){
            List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
            if (!results.isEmpty())
            {
                return results.get(0);
            }
        }
        return new PhotonPipelineResult();
    }

    public Optional<VisionEstimationResult> updateAndGetEstimatedPose() {
        Optional<VisionEstimationResult> result = Optional.empty();
        if(m_photonEstimator != null && m_camera != null) {
            PhotonPipelineResult latestResult = getLatestResult();
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            Pose3d referencePose = new Pose3d(Drive.getInstance().getPose());
            if (m_multiStrategy != null && latestResult.getTargets().size() > 1) {
                if (m_multiStrategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                    visionEst = m_photonEstimator.estimateCoprocMultiTagPose(latestResult);
                } else {
                    visionEst = m_photonEstimator.estimateRioMultiTagPose(latestResult, null, null);
                }
            } else {
                switch (m_singleStrategy) {
                    case LOWEST_AMBIGUITY:
                        visionEst = m_photonEstimator.estimateLowestAmbiguityPose(latestResult);
                        break;
                    case PNP_DISTANCE_TRIG_SOLVE:
                        visionEst = m_photonEstimator.estimatePnpDistanceTrigSolvePose(latestResult);
                        break;
                    case AVERAGE_BEST_TARGETS:
                        visionEst = m_photonEstimator.estimateAverageBestTargetsPose(latestResult);
                        break;
                    case CLOSEST_TO_CAMERA_HEIGHT:
                        visionEst = m_photonEstimator.estimateClosestToCameraHeightPose(latestResult);
                        break;
                    case CLOSEST_TO_REFERENCE_POSE:
                        // reference pose is just the current chassis pose, because earlier vision system didnt have a reference pose
                        visionEst = m_photonEstimator.estimateClosestToReferencePose(latestResult, referencePose);
                        break;
                    case CONSTRAINED_SOLVEPNP:
                        // TODO: oh my god thats so many values
                        visionEst = m_photonEstimator.estimateConstrainedSolvepnpPose(latestResult, null, null, null, false, 0);
                        break;
                    case CLOSEST_TO_LAST_POSE:
                        // same thing as reference pose
                        visionEst = m_photonEstimator.estimateClosestToReferencePose(latestResult, null);
                        break;
                    default:
                        // cant be reached but the compiler will give me a warning if i dont include this
                        break;
                }
            }

            if(visionEst.isPresent()) {
                EstimatedRobotPose est = visionEst.get();
                double ambiguity = getResultAmbiguity(est, latestResult);
                double latestTimestamp = latestResult.getTimestampSeconds();

                boolean valid = validateResult(est, ambiguity);
            
                boolean newResult = Math.abs(latestTimestamp - m_lastEstTime) > 1e-5;
                if (newResult) {
                    m_lastEstTime = latestTimestamp;
                }

                if(valid) {
                    Matrix<N3,N1> stdDevs = getEstimationStdDevs(est.estimatedPose.toPose2d());
                    result = Optional.of(new VisionEstimationResult(est.estimatedPose, latestTimestamp, ambiguity, stdDevs, latestResult));
                }
            }
        }
        m_latestResult = result;
        return m_latestResult;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;

        if(m_photonEstimator != null){
          var targets = getLatestResult().getTargets();
          int numTags = targets.size();
          double avgDist = 0;
          for (var tgt : targets) {
              var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
              if (tagPose.isEmpty()) continue;
              numTags++;
              avgDist +=
                      tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
          }
          if (numTags == 0) { return estStdDevs; }
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) { estStdDevs = VisionConstants.kMultiTagStdDevs; }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4) {
              estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
        }

        return estStdDevs;
    }

    // !! Currently only works for Multitag on Coproceesor, Update later to handle other stratgies !!
    public double getResultAmbiguity(EstimatedRobotPose estPose, PhotonPipelineResult latestResult) {
            double ambiguity = Double.MAX_VALUE;
            switch (estPose.strategy) {
                case MULTI_TAG_PNP_ON_COPROCESSOR:
                    //ambiguity = latestResult.getMultiTagResult().estimatedPose.ambiguity;
                    ambiguity = latestResult.getMultiTagResult().get().estimatedPose.ambiguity;
                    break;

                case LOWEST_AMBIGUITY:
                    var targets = estPose.targetsUsed;
                    for(PhotonTrackedTarget target : targets) {
                        if(target.getPoseAmbiguity() < ambiguity) {
                            ambiguity = target.getPoseAmbiguity();
                        }
                    }
                    break;
            
                default:
                    System.out.println("Unexpected Strategy Used For Pose Estimation. Returning Max Value of Double as Ambiguity");
                    break;
            }
            return ambiguity;
    }

    private boolean validateResult(EstimatedRobotPose estPose, double ambiguity) {
        if(ambiguity > VisionConstants.kMaxValidAmbiguity) {
            return false;
        }

        //Reject any poses that are outside the field
        if(estPose.estimatedPose.getX() < 0 ||
           estPose.estimatedPose.getX() > VisionConstants.kTagLayout.getFieldLength() ||
           estPose.estimatedPose.getY() < 0 ||
           estPose.estimatedPose.getY() > VisionConstants.kTagLayout.getFieldWidth()) {
            return false;
        }
        //Reject if robot is too too far from ground level
        if(Math.abs(estPose.estimatedPose.getZ()) > VisionConstants.kMaxZError) {
            return false;
        }
        //Reject if robot is tilted too much
        if(Math.abs(estPose.estimatedPose.getRotation().getX()) > VisionConstants.kMaxRollError ||
           Math.abs(estPose.estimatedPose.getRotation().getY()) > VisionConstants.kMaxPitchError) {
            return false;
        }

        return true;
    }

    public boolean shouldIncludeInPoseEstimates() { return m_includeInPoseEstimates; }

}