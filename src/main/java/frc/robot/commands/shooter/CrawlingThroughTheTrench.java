// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;
import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CrawlingThroughTheTrench extends Command {
  private final Shooter m_shooter;

  /** Creates a new CrawlingThroughTheTrench. */
  public CrawlingThroughTheTrench() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(Shooter.getInstance(), "Shooter", "CrawlingThroughTheTrench");
    m_shooter = Shooter.getInstance();
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setHoodTarget(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setTurretTarget(FieldUtils.getInstance().getAngleToPose(
      m_shooter.getTurretPose().toPose2d(),
      FieldUtils.getInstance().getClosestHubPose(m_shooter.getTurretPose().toPose2d()).toPose2d()).getRadians(), 0);
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
