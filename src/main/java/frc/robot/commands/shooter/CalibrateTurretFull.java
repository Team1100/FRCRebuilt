package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.TurretCalibration;
import frc.robot.testingdashboard.Command;

public class CalibrateTurretFull extends Command {
    private final Shooter m_Shooter;

    public CalibrateTurretFull() {
        super(Shooter.getInstance(), "Turret", "CalibrateTurretFull");

        m_Shooter = Shooter.getInstance();

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_Shooter.enableTurretCalibration(TurretCalibration.CALIBRATE_FULL);
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
