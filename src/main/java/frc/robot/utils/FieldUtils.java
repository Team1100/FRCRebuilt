// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.Constants.FieldLocationConstants;

public class FieldUtils{
    private static FieldUtils m_fieldUtils;
    private Timer timer;

    private Alliance m_alliance;

    // I wrote the directions based on their position on a face
    public static final AllianceAprilTags RedTags =
        new AllianceAprilTags(
            15,
            16,
            13,
            14,
            7,
            12,
            1,
            6,
            9,
            10,
            5,
            8,
            11,
            2,
            4,
            3);
    public static final AllianceAprilTags BlueTags = 
        new AllianceAprilTags(
            31,
            32,
            29,
            30,
            23,
            28,
            17,
            22,
            25, 
            26, 
            21,
            24,
            27,
            18,
            19,
            20);

    public static FieldUtils getInstance() {
        if(m_fieldUtils == null){
            m_fieldUtils = new FieldUtils();
        }
        return m_fieldUtils;
    }

    private FieldUtils(){
        timer = new Timer();
        m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public Alliance getAlliance() {
        m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return m_alliance;
    }

    public AllianceAprilTags getAllianceAprilTags(){
        if (m_alliance == Alliance.Red) return RedTags;
        else return BlueTags;
    }

    public Pose3d getTagPose(int tagId) {
        return VisionConstants.kTagLayout.getTagPose(tagId).get();
    }
    
    public Rotation2d getRotationOffset() {
        if (m_alliance == Alliance.Red) return Rotation2d.k180deg;
        else return Rotation2d.kZero;
    }

    public Rotation2d getAngleToPose(Pose2d currentPose, Pose2d targetPose) {
        Translation2d curTrans = currentPose.getTranslation();
        Translation2d targetTrans = targetPose.getTranslation();
        Translation2d toTarget = targetTrans.minus(curTrans);
        return toTarget.getAngle();
    }

    public boolean inAllianceHalf(Pose2d robotPose, Alliance alliance) {
        if (alliance == Alliance.Blue) return robotPose.getX() < FieldLocationConstants.kMidfieldX;
        else return robotPose.getX() > FieldLocationConstants.kMidfieldX;
    }

    public boolean inAllianceZone(Pose2d robotPose, Alliance alliance) {
        if (alliance == Alliance.Blue) return robotPose.getX() < FieldLocationConstants.kBlueAllianceZoneX;
        else return robotPose.getX() > FieldLocationConstants.kBlueAllianceZoneX;
    }

    public Pose3d getHubPose() {
        Transform3d tagToHub = new Transform3d(
            new Translation3d(-Units.inchesToMeters(23.5), 0, Units.inchesToMeters(18.0)), Rotation3d.kZero);
        return getTagPose(getAllianceAprilTags().frontRightHub).transformBy(tagToHub);
    }

    public enum AutoWinner {
        RED,
        BLUE
    }

    public AutoWinner getAutoWinner() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            return (gameData.charAt(0) == 'R') ? AutoWinner.RED : AutoWinner.BLUE;
        } else {
            return null;
        }
    }

    public Pose2d redPoseToAlliancePose(Pose2d pose) {
        AprilTagFieldLayout layout = Constants.VisionConstants.kTagLayout;
        return (m_alliance == Alliance.Red) ? pose :
                pose.rotateAround(new Translation2d(layout.getFieldLength()/2.0, layout.getFieldWidth()/2.0), Rotation2d.k180deg);
    }

    public Pose2d bluePoseToAlliancePose(Pose2d pose) {
        AprilTagFieldLayout layout = Constants.VisionConstants.kTagLayout;
        return (m_alliance == Alliance.Blue) ? pose :
                pose.rotateAround(new Translation2d(layout.getFieldLength()/2.0, layout.getFieldWidth()/2.0), Rotation2d.k180deg);
    }

    public enum GameState {
        AUTO,
        TRANSITION,
        RED_START,
        BLUE_START,
        ENDGAME
    }

    public GameState getGameState() {
        double time = timer.get();
        AutoWinner winner = getAutoWinner();
        GameState firstTeamHub;
        GameState secondTeamHub;
        if (winner == AutoWinner.RED){
            firstTeamHub = GameState.BLUE_START;
            secondTeamHub = GameState.RED_START;
        } else if (winner == AutoWinner.BLUE) {
            firstTeamHub = GameState.RED_START;
            secondTeamHub = GameState.BLUE_START;
        } else {
            firstTeamHub = null;
            secondTeamHub = null;
        }

        if (time > 0 && time < 20){
            return GameState.AUTO;
        } else if (time < 30) {
            return GameState.TRANSITION;
        } else if ((time < 55) || (time >= 80 && time < 105)) {
            return firstTeamHub;
        } else if ((time < 80) || (time >= 105 && time < 130)) {
            return secondTeamHub;
        } else if (time < 160) {
            return GameState.ENDGAME;
        } else {
            return null;
        }
    }

    public double stateTimeLeft() {
        double matchTime;
        if (DriverStation.isFMSAttached()) {
            matchTime = Timer.getMatchTime();
            if (DriverStation.isAutonomous()) {
                matchTime += 140;
            } else {
                matchTime += 20;
            }
        } else {
            matchTime = Timer.getFPGATimestamp();
        }

        if (matchTime<20) return 20-matchTime;
        if (matchTime<30) return 30-matchTime;
        if (matchTime<55) return 55-matchTime;
        if (matchTime<80) return 80-matchTime;
        if (matchTime<105) return 105-matchTime;
        if (matchTime<130) return 130-matchTime;
        return 160-matchTime;
    }

    public boolean inTrenchZone(Pose2d robotPose, double xVelocity)
    {
        double redTrenchZoneMin = c_fieldLength - c_trenchToDriverStationM - c_trenchZoneBaseWidthMeters;
        double redTrenchZoneMax = c_fieldLength - c_trenchToDriverStationM + c_trenchZoneBaseWidthMeters;
        double blueTrenchZoneMin = c_trenchToDriverStationM - c_trenchZoneBaseWidthMeters;
        double blueTrenchZoneMax = c_trenchToDriverStationM + c_trenchZoneBaseWidthMeters;

        if(xVelocity > 0)
        {
            blueTrenchZoneMin -= (xVelocity * c_trenchZoneVelocityScaling);
            redTrenchZoneMin -= (xVelocity * c_trenchZoneVelocityScaling);
        }
        else
        {
            blueTrenchZoneMax += (Math.abs(xVelocity) * c_trenchZoneVelocityScaling);
            redTrenchZoneMax += (Math.abs(xVelocity) * c_trenchZoneVelocityScaling);
        }

        double x = robotPose.getX();
        double y = robotPose.getY();
        return (
            (
                (x > redTrenchZoneMin && x < redTrenchZoneMax)
                || (x > blueTrenchZoneMin && x < blueTrenchZoneMax)
            ) &&
            (
                (y < c_trenchWidthMeters)
                || ( y > c_fieldWidth - c_trenchWidthMeters)
            )
        );
    }

    // Designates how wide of an area around the trench we want to designate as the "trench zone"
    private final double c_trenchZoneBaseWidthMeters = 0.25;
    private final double c_trenchZoneVelocityScaling = 0.75;

    private final double c_fieldLength = Constants.VisionConstants.kTagLayout.getFieldLength();
    private final double c_fieldWidth = Constants.VisionConstants.kTagLayout.getFieldWidth();
    private final double c_trenchWidthMeters = Units.inchesToMeters(62.65); 
    private final double c_trenchToDriverStationM = Units.inchesToMeters(182.11);

    public boolean isInField(Pose2d pose)
    {
        return (
            pose.getX() > 0 && 
            pose.getX() < c_fieldLength &&
            pose.getY() > 0 &&
            pose.getY() > c_fieldWidth
        );
    }
}