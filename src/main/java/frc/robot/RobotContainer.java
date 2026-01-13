// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.SwerveDrive;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TestingDashboard;

public class RobotContainer {
  private PowerDistribution m_pdBoard;

  private final OI m_OI;

  public RobotContainer() {
    RobotMap.init();

    m_pdBoard = new PowerDistribution(1, ModuleType.kRev);
    m_pdBoard.setSwitchableChannel(true);

    m_OI = OI.getInstance();

    registerCommands();

    Drive drive = Drive.getInstance();
    drive.setDefaultCommand(new SwerveDrive(m_OI.getDriveInputs()));
    
    configureBindings();

    TestingDashboard.getInstance().createTestingDashboard();
  }

  private void registerCommands() {

  }

  private void configureBindings() {
    m_OI.bindControls();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
