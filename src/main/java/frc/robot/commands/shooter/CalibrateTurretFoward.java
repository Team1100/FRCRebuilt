package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.TurretCalibration;
import frc.robot.testingdashboard.Command;

public class CalibrateTurretFoward extends Command {
    private final Shooter m_Shooter;

    public CalibrateTurretFoward() {
        super(Shooter.getInstance(), "Turret", "CalibrateTurretForward");

        m_Shooter = Shooter.getInstance();

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_Shooter.enableTurretCalibration(TurretCalibration.CALIBRATE_FORWARD);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return m_Shooter.isTurretCalibrating();
    }

    @Override
    public void end(boolean interrupted) {}
}
