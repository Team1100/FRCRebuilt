// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public class SwerveTurretOdometry3d extends SwerveDriveOdometry3d
{
    Transform3d m_baseToTurret;
    Rotation2d m_turretOffset;
    Rotation2d m_mostRecentTurretAngle;

    public SwerveTurretOdometry3d(   
      SwerveDriveKinematics kinematics,
      Rotation3d gyroAngle,
      Rotation2d turretOffset,
      Rotation2d initialTurretAngle,
      SwerveModulePosition[] modulePositions,
      Transform3d baseToTurretTransform,
      Pose3d initialRobotPose) {
        super(kinematics, gyroAngle, modulePositions, initialRobotPose);

        m_baseToTurret = baseToTurretTransform;
        m_turretOffset = turretOffset;
        m_mostRecentTurretAngle = initialTurretAngle;
    }

    @Override
    public Pose3d update(Rotation3d gyroAngle, SwerveModulePosition[] modulePositions)
    {
        super.update(gyroAngle, modulePositions);
        return getPoseMeters();
    }

    public Pose3d update(Rotation2d turretAngle, Rotation3d gyroAngle, SwerveModulePosition[] modulePositions)
    {
        m_mostRecentTurretAngle = turretAngle.rotateBy(m_turretOffset);
        return update(gyroAngle, modulePositions);
    }

    public Pose3d transformToChassisPose(Pose3d turretPose) {
        Rotation3d chassisRotation = turretPose.getRotation().rotateBy(new Rotation3d(m_mostRecentTurretAngle.unaryMinus()));
        Pose3d chassisRotatedTurretPose = new Pose3d(turretPose.getTranslation(), chassisRotation);
        return chassisRotatedTurretPose.transformBy(m_baseToTurret.inverse());
    }

    @Override
    public Pose3d getPoseMeters() {
        return super.getPoseMeters().transformBy(m_baseToTurret).transformBy(new Transform3d(new Translation3d(), new Rotation3d(m_mostRecentTurretAngle)));
    }
}
