package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.TurretCalibration;
import frc.robot.testingdashboard.Command;

public class StopTurretCalibration extends Command {
    private final Shooter m_Shooter;

    public StopTurretCalibration() {
        super(Shooter.getInstance(), "Turret", "StopTurretCalibration");

        m_Shooter = Shooter.getInstance();

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_Shooter.forceDisableTurretCalibration();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
