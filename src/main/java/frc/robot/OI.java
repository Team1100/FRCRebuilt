package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drive.JoystickHeadingDrive;
import frc.robot.subsystems.Drive;
import frc.robot.utils.SwerveDriveInputs;


public class OI {
    private static OI m_OI;

    private XboxController m_driverXboxController;
    private XboxController m_operatorXboxController;

    private SwerveDriveInputs m_driveInputs;

    public static OI getInstance() {
        if (m_OI == null) m_OI = new OI();
        return m_OI;
    }

    private OI() {
        m_driverXboxController = new XboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER);
        m_operatorXboxController = new XboxController(RobotMap.U_OPERATOR_XBOX_CONTROLLER);

        Supplier<Double> xInput;
        Supplier<Double> yInput;
        if (RobotBase.isReal()) {
            xInput = ()->m_driverXboxController.getLeftY();
            yInput = ()->m_driverXboxController.getLeftX();
        } else {
            xInput = ()->-m_driverXboxController.getLeftX();
            yInput = ()->m_driverXboxController.getLeftY();
        }
        m_driveInputs = new SwerveDriveInputs(xInput, yInput, ()->m_driverXboxController.getRightX());
    }

    public void bindControls() {
        new JoystickButton(m_driverXboxController, Button.kBack.value).onTrue(new InstantCommand(()->Drive.getInstance().zeroHeading()));
        new JoystickButton(m_driverXboxController, Button.kX.value).whileTrue(Drive.getInstance().sysIdQuasistatic(Direction.kForward));
        new JoystickButton(m_driverXboxController, Button.kA.value).whileTrue(Drive.getInstance().sysIdQuasistatic(Direction.kReverse));
        new JoystickButton(m_driverXboxController, Button.kY.value).whileTrue(Drive.getInstance().sysIdDynamic(Direction.kForward));
        new JoystickButton(m_driverXboxController, Button.kB.value).whileTrue(Drive.getInstance().sysIdDynamic(Direction.kReverse));
        new JoystickButton(m_driverXboxController, Button.kRightBumper.value).whileTrue(new JoystickHeadingDrive(m_driveInputs));
    }

    public XboxController getDriverController() {
        return m_driverXboxController;
    }

    public XboxController getOperatorController() {
        return m_operatorXboxController;
    }

    public SwerveDriveInputs getDriveInputs() {
        return m_driveInputs;
    }
}
