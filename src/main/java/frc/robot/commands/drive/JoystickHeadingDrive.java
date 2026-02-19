// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.SwerveDriveInputs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JoystickHeadingDrive extends Command {
  private SwerveDriveInputs m_DriveInputs;
  private PIDController m_headingController;
  private TDNumber m_TDheading;
  private double kPheading = 0.01;
  private double kIheading = 0;
  private double kDheading = 0;
  Drive m_drive;

  /** Creates a new Joystick Heading Drive. */
  public JoystickHeadingDrive(SwerveDriveInputs driveInputs) {
    super(Drive.getInstance(), "Basic", "JoystickHeadingDrive");
    m_drive = Drive.getInstance();
    m_DriveInputs = driveInputs;
    m_headingController = new PIDController(kPheading, kIheading, kDheading);
    m_headingController.enableContinuousInput(-180, 180);
    new TDSendable(m_drive, "Joystick Heading Drive", "Heading Controller", m_headingController);
    m_TDheading = new TDNumber(m_drive, "Joystick Heading Drive", "Target Heading", 0);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_TDheading.set(m_drive.getGyroAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationPower = 0.0;
    rotationPower = getRotationFromTransalation();
    m_drive.drive(
      -MathUtil.applyDeadband(m_DriveInputs.getX(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_DriveInputs.getY(), OIConstants.kDriveDeadband),
      rotationPower,
      true, false);
  }

  private double getRotationFromTransalation() {
    double x = -MathUtil.applyDeadband(m_DriveInputs.getX(), OIConstants.kDriveDeadband);
    double y = -MathUtil.applyDeadband(m_DriveInputs.getY(), OIConstants.kDriveDeadband);
    Rotation2d heading = new Rotation2d(x, y);
    double currentHeading = m_drive.getHeading();
    return m_headingController.calculate(currentHeading, heading.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
