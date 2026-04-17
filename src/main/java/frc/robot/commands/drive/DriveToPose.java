// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
// import frc.robot.commands.Lights.BlinkLights;
import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Lights.LightSection;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.drive.TargetPose;

public class DriveToPose extends Command {
  private Drive m_drive;
  private Supplier<TargetPose> m_targetSupplier;
  private Supplier<Pose2d> m_feedbackSupplier;
  private Pose2d m_targetPose;

  private ProfiledPIDController m_thetaController;
  private ProfiledPIDController m_driveXController;
  private ProfiledPIDController m_driveYController;

  TDNumber TDCurrentTargetX;
  TDNumber TDCurrentTargetY;
  TDNumber TDCurrentTargetAngle;

//   BlinkLights m_blinkLights;

  int m_periodicsAtGoal;

  public DriveToPose(Supplier<TargetPose> targetSupplier) {
    this(targetSupplier, Drive.getInstance()::getPose);
  }
  /** Creates a new TargetDrive. */
  public DriveToPose(Supplier<TargetPose> targetSupplier, Supplier<Pose2d> feedBackPoseSupplier) {
    super(Drive.getInstance(), "Drive", "DriveToPose");
    m_drive = Drive.getInstance();
    m_targetSupplier = targetSupplier;
    m_feedbackSupplier = feedBackPoseSupplier;

    TDCurrentTargetX = new TDNumber(m_drive, "Test Outputs", "Current Target X");
    TDCurrentTargetY = new TDNumber(m_drive, "Test Outputs", "Current Target Y");
    TDCurrentTargetAngle = new TDNumber(m_drive, "Test Outputs", "Current Target Angle");

    m_thetaController = new ProfiledPIDController(5.0, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveXController = new ProfiledPIDController(5, 0, 0, new Constraints(4.8, 4.8));
    m_driveYController = new ProfiledPIDController(5, 0, 0, new Constraints(4.8, 4.8));
  
    addRequirements(m_drive);

//     m_blinkLights = new BlinkLights(Constants.Color.red, LightSection.ACTIVE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_periodicsAtGoal = 0;

//     m_blinkLights.schedule();
    TargetPose target = m_targetSupplier.get();
    if (target != null && target.getPose() != null)
    {
      TDCurrentTargetX.set(target.getPose().getX());
      TDCurrentTargetY.set(target.getPose().getY());
      TDCurrentTargetAngle.set(target.getPose().getRotation().getDegrees());
      m_targetPose = target.getReversedApproach()? 
          new Pose2d(target.getPose().getX(), target.getPose().getY(), target.getPose().getRotation().plus(Rotation2d.k180deg)) 
          : target.getPose();
      Pose2d currentPose = m_feedbackSupplier.get();
      m_thetaController.reset(currentPose.getRotation().getRadians());
      m_driveXController.reset(currentPose.getX());
      m_driveYController.reset(currentPose.getY());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_targetPose);
    if(m_targetPose != null)
    {
      Pose2d currentPose = m_feedbackSupplier.get();
      double omega = m_thetaController.calculate(currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());
      double xVel = m_driveXController.calculate(currentPose.getX(), m_targetPose.getX());
      double yVel = m_driveYController.calculate(currentPose.getY(), m_targetPose.getY());

      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, omega,  currentPose.getRotation());
      m_drive.drive(speeds);

    //  if ((m_feedbackSupplier.get().getTranslation().getDistance(m_targetPose.getTranslation()) < Constants.AutoConstants.kTranslationTolerance) &&
    //   (Math.abs(m_feedbackSupplier.get().getRotation().minus(m_targetPose.getRotation()).getRadians()) < Constants.AutoConstants.kThetaTolerance))
    //     m_periodicsAtGoal++;
    //   else m_periodicsAtGoal = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(new ChassisSpeeds());
//     m_blink5.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // (m_periodicsAtGoal >= Constants.AutoConstants.kReefFinishedPeriodics);
  }
}