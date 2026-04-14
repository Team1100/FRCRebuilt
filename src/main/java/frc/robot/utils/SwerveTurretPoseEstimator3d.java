// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

/** Add your docs here. */
public class SwerveTurretPoseEstimator3d extends PoseEstimator3d<SwerveModulePosition[]> 
{
    private final int m_numModules;
    private SwerveTurretOdometry3d m_odometry;

    public SwerveTurretPoseEstimator3d(      
      SwerveDriveKinematics kinematics,
      Rotation3d gyroAngle,
      Rotation2d turretOffset,
      Rotation2d turretAngle,
      SwerveModulePosition[] modulePositions,
      Pose3d initialPoseMeters,
      Transform3d baseToTurretTrans) {
    this(
        kinematics,
        gyroAngle,
        turretAngle,
        modulePositions,
        initialPoseMeters,
        baseToTurretTrans,
        VecBuilder.fill(0.1, 0.1, 0.1, 0.1),
        VecBuilder.fill(0.9, 0.9, 0.9, 0.9),
        new SwerveTurretOdometry3d(
            kinematics,
            gyroAngle,
            turretOffset,
            turretAngle,
            modulePositions, 
            baseToTurretTrans, 
            initialPoseMeters));
  }

  public SwerveTurretPoseEstimator3d(
      SwerveDriveKinematics kinematics,
      Rotation3d gyroAngle,
      Rotation2d turretAngle,
      SwerveModulePosition[] modulePositions,
      Pose3d initialPoseMeters,
      Transform3d baseToTurretTrans,
      Matrix<N4, N1> stateStdDevs,
      Matrix<N4, N1> visionMeasurementStdDevs,
      SwerveTurretOdometry3d odometry) {
    super(
        kinematics,
        odometry,
        stateStdDevs,
        visionMeasurementStdDevs);

    m_odometry = odometry;
    m_numModules = modulePositions.length;
  }

  public Pose3d getEstimatedChassisPose() {
    Pose3d turretPose = getEstimatedPosition();
    return m_odometry.transformToChassisPose(turretPose);
  }

  public Pose3d update(Rotation3d gyroAngle, Rotation2d turretAngle, SwerveModulePosition[] wheelPositions) {
        return updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, turretAngle, wheelPositions);
    }

  public Pose3d updateWithTime(
      double currentTimeSeconds, Rotation3d gyroAngle, Rotation2d turretAngle, SwerveModulePosition[] wheelPositions) {
    if (wheelPositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }

    m_odometry.update(turretAngle, gyroAngle, wheelPositions);
    return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
  }

}
