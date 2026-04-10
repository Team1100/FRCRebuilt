package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;

public class HoodCalibrate extends Command {
    private final Shooter m_Shooter;
    private double m_startTime;

    public HoodCalibrate() {
        super(Shooter.getInstance(), "Hood", "HoodCalibrate");
        m_Shooter = Shooter.getInstance();
        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        m_Shooter.setHoodTarget(-35);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > m_startTime + 0.5;
    }
}
