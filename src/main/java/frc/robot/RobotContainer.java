// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.SwerveDrive;
import frc.robot.commands.shooter.ShootToPose;
import frc.robot.commands.vision.DisablePoseUpdates;
import frc.robot.commands.vision.EnablePoseUpdates;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.TestingDashboard;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.drive.MAXSwerveModule;

public class RobotContainer {
  private PowerDistribution m_pdBoard;

  private final OI m_OI;

  public RobotContainer() {

    m_pdBoard = new PowerDistribution(1, ModuleType.kRev);
    m_pdBoard.setSwitchableChannel(true);

    m_OI = OI.getInstance();

    registerCommands();

    Climber.getInstance();
    
    Drive drive = Drive.getInstance();
    drive.setDefaultCommand(new SwerveDrive(m_OI.getDriveInputs()));

    Intake.getInstance();

    LED.getInstance();

    //MAXSwerveModule.getInstance(); the method doesnt exist?
  
    Shooter.getInstance();

    Spindexer.getInstance();

    Vision.getInstance();
    
    configureBindings();

    TestingDashboard.getInstance().createTestingDashboard();
  }

  private void registerCommands() {
    new DisablePoseUpdates();
    new EnablePoseUpdates();

    NamedCommands.registerCommand("ShootToHub", new ShootToPose(FieldUtils.getInstance()::getHubPose));
  }

  private void configureBindings() {
    m_OI.bindControls();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
