package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drive.JoystickHeadingDrive;
import frc.robot.commands.intake.DeployerIn;
import frc.robot.commands.intake.DeployerOut;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeAAAAA;
import frc.robot.commands.shooter.StopTurretCalibration;
import frc.robot.commands.shooter.CalibrateTurretFull;
import frc.robot.commands.shooter.ChimneyDown;
import frc.robot.commands.shooter.ChimneyUp;
import frc.robot.commands.shooter.ManualShooterControl;
import frc.robot.commands.shooter.RealManualTurretControl;
import frc.robot.commands.shooter.ShootMap;
import frc.robot.commands.shooter.ShootSpecified;
import frc.robot.commands.shooter.ShootToPose;
import frc.robot.commands.spindexer.SpindexerReverse;
import frc.robot.commands.spindexer.SpindexerSpin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.TriggerBuilder;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.Referrable;
import frc.robot.utils.drive.SwerveDriveInputs;
import frc.robot.utils.TriggerBuilder.RumbleIndicator;
import frc.robot.utils.TriggerBuilder.PrintIndicator;
import frc.robot.utils.TriggerBuilder.SwitchIndicator;


public class OI {
    private static OI m_OI;

    private CommandXboxController m_driverXboxController;
    private CommandXboxController m_operatorXboxController;

    private SwerveDriveInputs m_driveInputs;

	private enum Submap {
		DRIVE_SYSID,
		AUTO,
		MANUAL;
	}

	private final Referrable<Submap> m_operatorSubmap = new Referrable<Submap>(Submap.AUTO);
	private final Referrable<Submap> m_driverSubmap = new Referrable<Submap>(Submap.MANUAL);

	//private final TDSendable m_operatorSubmapSendable = new TDSendable(OI, null, null, getDriverController())

    public static OI getInstance() {
        if (m_OI == null) m_OI = new OI();
        return m_OI;
    }

    private OI() {
        m_driverXboxController = new CommandXboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER);
        m_operatorXboxController = new CommandXboxController(RobotMap.U_OPERATOR_XBOX_CONTROLLER);

        Supplier<Double> xInput;
        Supplier<Double> yInput;
        if (RobotBase.isReal()) {
            xInput = m_driverXboxController::getLeftY;
            yInput = m_driverXboxController::getLeftX;
        } else {
            xInput = m_driverXboxController::getLeftX;
            yInput = m_driverXboxController::getLeftY;
        }
        m_driveInputs = new SwerveDriveInputs(xInput, yInput, m_driverXboxController::getRightX);
    }

	public void bindControls() {
		//SwitchIndicator driverIndicator = new RumbleIndicator(m_driverXboxController.getHID());
		SwitchIndicator driverIndicator = new PrintIndicator();
		new TriggerBuilder<Submap>(m_driverSubmap)
			.beginSubmap(Submap.DRIVE_SYSID)
				.whileTrue(m_driverXboxController.x(), Drive.getInstance().sysIdQuasistatic(Direction.kForward))
				.whileTrue(m_driverXboxController.a(), Drive.getInstance().sysIdQuasistatic(Direction.kReverse))
				.whileTrue(m_driverXboxController.y(), Drive.getInstance().sysIdDynamic(Direction.kForward))
				.whileTrue(m_driverXboxController.b(), Drive.getInstance().sysIdDynamic(Direction.kReverse))

				.switchSubmap(driverIndicator, m_driverXboxController.start(), Submap.AUTO)
			.endSubmap()

			.onTrue(m_driverXboxController.back(), new InstantCommand(()->Drive.getInstance().zeroHeading()))

			.beginSubmap(Submap.MANUAL)
				/*
				.whileTrue(m_driverXboxController.leftTrigger(), new IntakeIn())
				.whileTrue(m_driverXboxController.rightTrigger(), new IntakeOut())

				.whileTrue(m_driverXboxController.leftBumper(), new SpindexerSpin())

				.whileTrue(m_driverXboxController.b(), new ChimneyUp())
				*/

				.whileTrue(m_driverXboxController.y(), new DeployerOut())
				.whileTrue(m_driverXboxController.x(), new DeployerIn())

				.whileTrue(m_driverXboxController.leftBumper(), new IntakeIn())
				.whileTrue(m_driverXboxController.rightBumper(), new IntakeAAAAA())

				.whileTrue(m_driverXboxController.leftTrigger(), new IntakeAAAAA())
				.whileTrue(m_driverXboxController.rightTrigger(), new JoystickHeadingDrive(m_driveInputs))

				.whileTrue(m_driverXboxController.povUp(), Commands.startEnd(
					()->{Climber.getInstance().setClimberTargetAngle(5);},
					()->{Climber.getInstance().setClimberTargetAngle(0);}
				))
				//.onTrue(m_driverXboxController.povUp(), Commands.print("you're not crazy yet"))
				
				.switchSubmap(driverIndicator, m_driverXboxController.start(), Submap.AUTO)
			.endSubmap()

			.beginSubmap(Submap.AUTO)
				.switchSubmap(driverIndicator, m_driverXboxController.start(), Submap.MANUAL)
			.endSubmap()

			.register();

		SwitchIndicator operatorIndicator = new RumbleIndicator(m_operatorXboxController.getHID());
		new TriggerBuilder<Submap>(m_operatorSubmap)
			.beginSubmap(Submap.AUTO)
				.whileTrue(m_operatorXboxController.rightBumper(), Commands.parallel(
					new ChimneyUp(),
					new SpindexerSpin()
				))

				.whileTrue(m_operatorXboxController.leftBumper(), Commands.parallel(
					new ChimneyDown(),
					new SpindexerReverse()
				))

				// midfield
				.whileTrue(m_operatorXboxController.povUp(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(2.5,4), Rotation2d.kZero), 3000, -18, 0),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(2.5,4), Rotation2d.kZero), 3000, -18, 0),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// left trench
				.whileTrue(m_operatorXboxController.povUpLeft(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCW_90deg), 3000, -21.8, 0.13),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCW_90deg), 3000, -21.8, 0.13),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// right trench
				.whileTrue(m_operatorXboxController.povUpRight(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCCW_90deg), 3000, -21.8, -0.35),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCCW_90deg), 3000, -21.8, -0.35),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))

				// ladder
				.whileTrue(m_operatorXboxController.povDown(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(1.5,4), Rotation2d.kZero), 3100, -25, 0),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(1.5,4), Rotation2d.kZero), 3100, -25, 0),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// left corner
				.whileTrue(m_operatorXboxController.povDownLeft(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,7.5), Rotation2d.kZero), 3400, -35, -0.77),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,7.5), Rotation2d.kZero), 3400, -35, -0.77),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// right corner
				.whileTrue(m_operatorXboxController.povDownRight(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,0.5), Rotation2d.kZero), 3400, -35, 0.59),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,0.5), Rotation2d.kZero), 3400, -35, 0.59),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))

				// ferry 0
				.whileTrue(m_operatorXboxController.y(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kZero), 4000, -35, 0),
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kZero), 4000, -35, 0),
					Alliance.Blue,
					ShootMap.Target.FERRY
				))
				// ferry clockwise 90
				.whileTrue(m_operatorXboxController.b(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCW_90deg), 4000, -35, Math.PI/2),
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCW_90deg), 4000, -35, Math.PI/2),
					Alliance.Blue,
					ShootMap.Target.FERRY
				))
				// ferry counter clockwise 90
				.whileTrue(m_operatorXboxController.x(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCCW_90deg), 4000, -35, -Math.PI/2),
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCCW_90deg), 4000, -35, -Math.PI/2),
					Alliance.Blue,
					ShootMap.Target.FERRY
				))

				.switchSubmap(operatorIndicator, m_operatorXboxController.start(), Submap.MANUAL)
			.endSubmap()

			.beginSubmap(Submap.MANUAL)
				.onTrue(m_operatorXboxController.a(), Commands.runOnce(()->{Shooter.getInstance().setTurretRobotRelative(true);}, Shooter.getInstance()))
				.onTrue(m_operatorXboxController.y(), Commands.runOnce(()->{Shooter.getInstance().setTurretRobotRelative(false);}, Shooter.getInstance()))
				.onTrue(m_operatorXboxController.x(), new StopTurretCalibration())
				.onTrue(m_operatorXboxController.b(), new CalibrateTurretFull())

				.map(m_operatorXboxController.povUp(), new RealManualTurretControl(), Trigger::toggleOnTrue)
				.map(m_operatorXboxController.povLeft(), new ManualShooterControl(), Trigger::toggleOnTrue)

				.onTrue(m_operatorXboxController.povDown(), Commands.runOnce(()->{Shooter.getInstance().forceTurretZero();}, Shooter.getInstance()))

				.switchSubmap(operatorIndicator, m_operatorXboxController.start(), Submap.AUTO)
			.endSubmap()

			.register();
    }

    public XboxController getDriverController() {
        return m_driverXboxController.getHID();
    }

    public XboxController getOperatorController() {
        return m_operatorXboxController.getHID();
    }

    public SwerveDriveInputs getDriveInputs() {
        return m_driveInputs;
    }
}
