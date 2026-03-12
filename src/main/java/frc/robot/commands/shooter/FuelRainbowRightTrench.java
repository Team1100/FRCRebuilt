package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;

public class FuelRainbowRightTrench extends Command {
    private final Shooter m_Shooter;

    public FuelRainbowRightTrench() {
        super(Shooter.getInstance(), "Shooting", "Fuel Rainbow Right Trench");

        m_Shooter = Shooter.getInstance();

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_Shooter.setFlywheelTarget(3000);
        m_Shooter.setHoodTarget(-21.8);
        m_Shooter.setTurretTarget(-0.35, 0);
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
