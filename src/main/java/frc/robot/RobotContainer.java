// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveToTDPose;
import frc.robot.commands.drive.SlowSwerveDrive;
import frc.robot.commands.drive.SwerveDrive;
import frc.robot.commands.drive.YoureUnderArrest;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.IntakeOscillate;
import frc.robot.commands.intake.IntakeOutake;
import frc.robot.commands.intake.DeployerOut;
import frc.robot.commands.shooter.ChimneyUp;
import frc.robot.commands.shooter.CrawlingThroughTheTrench;
import frc.robot.commands.shooter.FerryShoot;
import frc.robot.commands.shooter.FuelRainbowHub;
import frc.robot.commands.shooter.FuelRainbowLeftTrench;
import frc.robot.commands.shooter.FuelRainbowRightTrench;
import frc.robot.commands.shooter.ShootToPose;
import frc.robot.commands.spindexer.SpindexerSpin;
import frc.robot.commands.vision.DisablePoseUpdates;
import frc.robot.commands.vision.EnablePoseUpdates;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.testingdashboard.TestingDashboard;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.drive.MAXSwerveModule;

public class RobotContainer {
  private PowerDistribution m_pdBoard;

  private final OI m_OI;
  private final SendableChooser<Command> m_autoChooser;

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
  
    Shooter shooter = Shooter.getInstance();
    shooter.setDefaultCommand(new CrawlingThroughTheTrench());

    Spindexer.getInstance();

    Vision.getInstance();

    m_autoChooser = AutoBuilder.buildAutoChooser("NoAuto");
    m_autoChooser.addOption("NoAuto", Commands.print("No auto selected"));
    new TDSendable(Drive.getInstance(), "Autos", "Chooser", m_autoChooser);
    
    configureBindings();

    TestingDashboard.getInstance().createTestingDashboard();
    SmartDashboard.putData(m_autoChooser);
  }

  private void registerCommands() {
    new DisablePoseUpdates();
    new EnablePoseUpdates();

    NamedCommands.registerCommand("ShootToHub", new ShootToPose(FieldUtils.getInstance()::getHubPose));

    new YoureUnderArrest();
    new FuelRainbowHub();
    new FuelRainbowLeftTrench();
    new FuelRainbowRightTrench();

    new FerryShoot();
    new CrawlingThroughTheTrench();

    new IntakeIn();
    new IntakeOscillate();
    new DeployerOut();

    new SpindexerSpin();
    new ChimneyUp();

    new SlowSwerveDrive(null);

    new DriveToTDPose();
  }

  private void configureBindings() {
    m_OI.bindControls();
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
