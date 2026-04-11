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
import frc.robot.Constants.ControllerLayout;
import frc.robot.commands.drive.JoystickHeadingDrive;
import frc.robot.commands.drive.SlowSwerveDrive;
import frc.robot.commands.intake.DeployerIn;
import frc.robot.commands.intake.DeployerOut;
import frc.robot.commands.intake.IntakeAAAAA;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOscillate;
import frc.robot.commands.shooter.StopTurretCalibration;
import frc.robot.commands.shooter.CalibrateTurretFull;
import frc.robot.commands.shooter.ChimneyDown;
import frc.robot.commands.shooter.ChimneyUp;
import frc.robot.commands.shooter.FerryShoot;
import frc.robot.commands.shooter.HoodCalibrate;
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
	private final Referrable<Submap> m_driverSubmap = new Referrable<Submap>(Submap.AUTO);

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
            xInput = ()->-m_driverXboxController.getLeftX();
            yInput = m_driverXboxController::getLeftY;
        }
        m_driveInputs = new SwerveDriveInputs(xInput, yInput, m_driverXboxController::getRightX);
    }

	public void bindControls() {
		if (Constants.CONTROLLER_LAYOUT == ControllerLayout.COMPETITION) {
			bindCompetitionLayout(m_driverXboxController, m_operatorXboxController);
		} else {
			bindDebugLayout(m_driverXboxController);
		}
    }
	
	public void bindCompetitionLayout(CommandXboxController driver, CommandXboxController operator) {
		SwitchIndicator driverIndicator = new RumbleIndicator(driver.getHID());
		new TriggerBuilder<>(m_driverSubmap)
			.onTrue(driver.back(), new InstantCommand(()->Drive.getInstance().zeroHeading()))

			.beginSubmap(Submap.AUTO)
				.whileTrue(driver.leftBumper(), new IntakeIn())
				.whileTrue(driver.povUp(), new IntakeOscillate())
				.whileTrue(driver.rightBumper(), new IntakeAAAAA())
				.whileTrue(driver.b(), new DeployerOut())
				.whileTrue(driver.rightTrigger(), new DeployerIn())

				.whileTrue(driver.leftTrigger(), new SlowSwerveDrive(m_driveInputs))

				.switchSubmap(driverIndicator, driver.start(), Submap.MANUAL)
			.endSubmap()

			.beginSubmap(Submap.MANUAL)
				.switchSubmap(driverIndicator, driver.start(), Submap.AUTO)
			.endSubmap()

			.register();

		SwitchIndicator operatorIndicator = new RumbleIndicator(operator.getHID());
		new TriggerBuilder<Submap>(m_operatorSubmap)
			.beginSubmap(Submap.AUTO)
				.whileTrue(operator.rightBumper(), Commands.parallel(new SpindexerSpin(), new ChimneyUp()))

				.whileTrue(operator.leftBumper(), Commands.parallel(new SpindexerReverse(), new ChimneyDown()))

				// midfield
				.whileTrue(operator.povUp(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(2.5,4), Rotation2d.kZero), 3000, -18, 0),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(2.5,4), Rotation2d.kZero), 3000, -18, 0),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// left trench
				.whileTrue(operator.povUpLeft(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCW_90deg), 3000, -21.8, 0.13),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCW_90deg), 3000, -21.8, 0.13),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// right trench
				.whileTrue(operator.povUpRight(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCCW_90deg), 3000, -21.8, -0.35),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32,7.5), Rotation2d.kCCW_90deg), 3000, -21.8, -0.35),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))

				// ladder
				.whileTrue(operator.povDown(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(1.5,4), Rotation2d.kZero), 3100, -25, 0),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(1.5,4), Rotation2d.kZero), 3100, -25, 0),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// left corner
				.whileTrue(operator.povDownLeft(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,7.5), Rotation2d.kZero), 3400, -35, -0.77),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,7.5), Rotation2d.kZero), 3400, -35, -0.77),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))
				// right corner
				.whileTrue(operator.povDownRight(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,0.5), Rotation2d.kZero), 3400, -35, 0.59),
					new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5,0.5), Rotation2d.kZero), 3400, -35, 0.59),
					Alliance.Blue,
					ShootMap.Target.SHOOT_HUB
				))

				// ferry 0
				.whileTrue(operator.y(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kZero), 6000, -40, 0),
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kZero), 6000, -40, 0),
					Alliance.Blue,
					ShootMap.Target.FERRY
				))
				// ferry clockwise 90
				/*.whileTrue(operator.b(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCW_90deg), 6000, -40, Math.PI/2),
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCW_90deg), 6000, -40, Math.PI/2),
					Alliance.Blue,
					ShootMap.Target.FERRY
				))*/
				// ferry counter clockwise 90
				/*.whileTrue(operator.x(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCCW_90deg), 6000, -40, -Math.PI/2),
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kCCW_90deg), 6000, -40, -Math.PI/2),
					Alliance.Blue,
					ShootMap.Target.FERRY
				))*/
				// low power loser ferry
				.whileTrue(operator.b(), new ShootMap(
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kZero), 4500, -40, -Math.PI/2),
					new ShootMap.ShootMapSetpoint(new Pose2d(Translation2d.kZero, Rotation2d.kZero), 4500, -40, -Math.PI/2),
					Alliance.Blue,
					ShootMap.Target.FERRY
				))

				.whileTrue(operator.a(), new ShootToPose(FieldUtils.getInstance()::getHubPose))

				.whileTrue(operator.x(), new FerryShoot())

				.switchSubmap(operatorIndicator, operator.start(), Submap.MANUAL)
			.endSubmap()

			.beginSubmap(Submap.MANUAL)
				.onTrue(operator.a(), Commands.runOnce(()->{Shooter.getInstance().setTurretRobotRelative(true);}, Shooter.getInstance()))
				.onTrue(operator.y(), Commands.runOnce(()->{Shooter.getInstance().setTurretRobotRelative(false);}, Shooter.getInstance()))
				.onTrue(operator.x(), new StopTurretCalibration())
				.onTrue(operator.b(), new CalibrateTurretFull())


				.whileTrue(operator.rightTrigger(), Commands.parallel(
					new ChimneyUp(),
					new SpindexerSpin()
				))
				.whileTrue(operator.leftTrigger(), Commands.parallel(
					new ChimneyDown(),
					new SpindexerReverse()
				))


				.map(operator.povUp(), new RealManualTurretControl(), Trigger::toggleOnTrue)
				.map(operator.povLeft(), new ManualShooterControl(), Trigger::toggleOnTrue)

				.onTrue(operator.povDown(), Commands.runOnce(()->{Shooter.getInstance().forceTurretZero();}, Shooter.getInstance()))

				.switchSubmap(operatorIndicator, operator.start(), Submap.AUTO)
			.endSubmap()

			.register();
	}

	public void bindDebugLayout(CommandXboxController driver) {
		SwitchIndicator driverIndicator = new RumbleIndicator(driver.getHID());
		new TriggerBuilder<>(m_driverSubmap)
			.beginSubmap(Submap.DRIVE_SYSID)
				.whileTrue(driver.x(), Drive.getInstance().sysIdQuasistatic(Direction.kForward))
				.whileTrue(driver.a(), Drive.getInstance().sysIdQuasistatic(Direction.kReverse))
				.whileTrue(driver.y(), Drive.getInstance().sysIdDynamic(Direction.kForward))
				.whileTrue(driver.b(), Drive.getInstance().sysIdDynamic(Direction.kReverse))

				.switchSubmap(driverIndicator, driver.start(), Submap.AUTO)
			.endSubmap()

			.onTrue(driver.back(), new InstantCommand(()->Drive.getInstance().zeroHeading()))

			.beginSubmap(Submap.AUTO)
				.whileTrue(driver.leftBumper(), new IntakeIn())
				.whileTrue(driver.povUp(), new IntakeOscillate())
				.whileTrue(driver.rightBumper(), new IntakeAAAAA())
				.whileTrue(driver.povLeft(), new DeployerIn())
				.whileTrue(driver.povRight(), new DeployerOut())

				.whileTrue(driver.a(), new ShootToPose(FieldUtils.getInstance()::getHubPose))
				.whileTrue(driver.x(), new FerryShoot())

				.whileTrue(driver.leftTrigger(), new SlowSwerveDrive(m_driveInputs))

				.whileTrue(driver.rightTrigger(), new SpindexerSpin())

				.switchSubmap(driverIndicator, driver.start(), Submap.MANUAL)
			.endSubmap()

			.beginSubmap(Submap.MANUAL)
				.onTrue(m_driverXboxController.b(), new CalibrateTurretFull())
				.onTrue(m_driverXboxController.a(), new HoodCalibrate())

				.switchSubmap(driverIndicator, driver.start(), Submap.AUTO)
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
