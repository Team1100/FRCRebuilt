// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldLocationConstants;

public class FieldUtils{
    private static FieldUtils m_fieldUtils;
    private Timer timer;

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

    public static FieldUtils getInstance(){
        if(m_fieldUtils == null){
            m_fieldUtils = new FieldUtils();
        }
        return m_fieldUtils;
    }

    private FieldUtils(){
        timer = new Timer();
    }

    public AllianceAprilTags getAllianceAprilTags(){
        AllianceAprilTags tags = BlueTags;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Red){
                tags = RedTags;
            } else if(alliance.get() == DriverStation.Alliance.Blue) {
                tags = BlueTags;
            }
        }
        return tags;
    }

    public Pose3d getTagPose(int tagId)
    {
        return VisionConstants.kTagLayout.getTagPose(tagId).get();
    }
    
    public Rotation2d getRotationOffset() {
        Rotation2d offset = new Rotation2d();//returns no offset by default
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && 
            alliance.get() == DriverStation.Alliance.Red){
                offset = new Rotation2d(Math.PI);
        }
        return offset;
    }

    public Rotation2d getAngleToPose(Pose2d currentPose, Pose2d targetPose) {
        Translation2d curTrans = currentPose.getTranslation();
        Translation2d targetTrans = targetPose.getTranslation();
        Translation2d toTarget = targetTrans.minus(curTrans);
        return toTarget.getAngle();
    }

    public boolean inAllianceHalf(Pose2d robotPose, Alliance alliance) {
        return (alliance == Alliance.Blue) ^ (robotPose.getX() > FieldLocationConstants.kMidfieldX);
    }


    public char getAutoWinner(){ 
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0){
            return (gameData.charAt(0)); // R or B
        } else {
            return 'n'; //null
        }
    }
    public void startGameTimer(){
        timer.start();
    }
    public char getGameState(){
        double time = timer.get();
        char firstTeam;
        char secondTeam = getAutoWinner();
        if (secondTeam == 'R'){
            firstTeam = 'B';
        } else if (secondTeam == 'B') {
            firstTeam = 'R';
        } else {
            firstTeam = 'n'; //null
        }

        if (time > 0 && time < 20){
            return 'A'; //auto
        } else if (time >= 20 && time < 30){
            return 'T'; //transition
        } else if (time >= 30 && time < 55){
            return firstTeam;
        } else if (time >= 55 && time < 80){
            return secondTeam;
        } else if (time >= 80 && time < 105){
            return firstTeam;
        } else if (time >= 105 && time < 130){
            return secondTeam;
        } else if (time >= 130 && time < 160){
            return 'E'; //endgame
        } else {
            return 'n'; //null
        }

    }
    public double stateTimeLeft(){
        return timer.getMatchTime(); //approximate
    }
    
}