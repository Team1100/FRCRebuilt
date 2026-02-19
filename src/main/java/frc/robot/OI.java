package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drive;
import frc.robot.utils.TriggerBuilder;
import frc.robot.utils.Referrable;
import frc.robot.utils.SwerveDriveInputs;
import frc.robot.utils.TriggerBuilder.RumbleIndicator;
import frc.robot.utils.TriggerBuilder.SwitchIndicator;


public class OI {
    private static OI m_OI;

    private CommandXboxController m_driverXboxController;
    private CommandXboxController m_operatorXboxController;

    private SwerveDriveInputs m_driveInputs;

	private enum Submap {
		AUTO,
		MANUAL;
	}

	private final Referrable<Submap> m_operatorSubmap = new Referrable<Submap>(Submap.AUTO);
	private final Referrable<Submap> m_driverSubmap = new Referrable<Submap>(Submap.AUTO);

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
		SwitchIndicator driverIndicator = new RumbleIndicator(m_driverXboxController.getHID());
		new TriggerBuilder<Submap>(m_driverSubmap)
			.whileTrue(m_driverXboxController.x(), Drive.getInstance().sysIdQuasistatic(Direction.kForward))
			.whileTrue(m_driverXboxController.a(), Drive.getInstance().sysIdQuasistatic(Direction.kReverse))
			.whileTrue(m_driverXboxController.y(), Drive.getInstance().sysIdDynamic(Direction.kForward))
			.whileTrue(m_driverXboxController.b(), Drive.getInstance().sysIdDynamic(Direction.kReverse))
			.onTrue(m_driverXboxController.back(), new InstantCommand(()->Drive.getInstance().zeroHeading()))

			.beginSubmap(Submap.AUTO)
				.switchSubmap(driverIndicator, m_driverXboxController.start(), Submap.MANUAL)
			.endSubmap()

			.beginSubmap(Submap.MANUAL)
				.switchSubmap(driverIndicator, m_driverXboxController.start(), Submap.AUTO)
			.endSubmap()

			.register();

		SwitchIndicator operatorIndicator = new RumbleIndicator(m_operatorXboxController.getHID());
		new TriggerBuilder<Submap>(m_operatorSubmap)
			.beginSubmap(Submap.AUTO)
				.whileTrue(m_operatorXboxController.a(), Commands.print("Auto A"))
				.onTrue(m_operatorXboxController.b(), Commands.print("Auto B"))

				.switchSubmap(operatorIndicator, m_operatorXboxController.start(), Submap.MANUAL)
			.endSubmap()

			.beginSubmap(Submap.MANUAL)
				.whileTrue(m_operatorXboxController.a(), Commands.print("Manual A"))
				.onTrue(m_operatorXboxController.b(), Commands.print("Manual B"))

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
