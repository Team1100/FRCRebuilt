// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.subsystems.Climber;
import frc.robot.testingdashboard.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ascend extends Command {
  /** Creates a new Ascend. */
  Climber m_Climber;

  public Ascend() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(Climber.getInstance(), "Climber", "Ascend");

    m_Climber = Climber.getInstance();

    addRequirements(m_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.FunnyClimbAscendAction();
  }

  // Called once the command ends or is inimport frc.robot.utils.Configuration;terrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climber.FunnyClimbHaltAction();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
