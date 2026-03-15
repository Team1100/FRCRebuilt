package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;

public class FuelRainbowHub extends Command {
    private final Shooter m_Shooter;

    public FuelRainbowHub() {
        super(Shooter.getInstance(), "Shooting", "FuelRainbowHub");

        m_Shooter = Shooter.getInstance();

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_Shooter.setFlywheelTarget(3400);
        m_Shooter.setHoodTarget(-5);
        m_Shooter.setTurretTarget(0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.setHoodTarget(0);
        m_Shooter.setFlywheelTarget(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
