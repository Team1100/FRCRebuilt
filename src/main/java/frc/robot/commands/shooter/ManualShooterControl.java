package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;

public class ManualShooterControl extends Command{
    private final Shooter m_Shooter;
    private final OI m_OI;

    private double m_hoodAngle, m_flywheelSpeed, m_turretAngle;

    public ManualShooterControl() {
        super(Shooter.getInstance(), "Controls", "ManualShooterControl");

        m_Shooter = Shooter.getInstance();
        m_OI = OI.getInstance();
        
        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_hoodAngle = 0;
        m_flywheelSpeed = 0;
        m_turretAngle = m_Shooter.getTurretTarget();
    }

    @Override
    public void execute() {
        XboxController operator = m_OI.getOperatorController();

        m_turretAngle += MathUtil.applyDeadband(operator.getLeftX(), 0.05) * Constants.schedulerPeriodTime * 0.5;
        m_turretAngle = m_turretAngle%Math.PI;
        m_hoodAngle += MathUtil.applyDeadband(operator.getRightY(), 0.05) * Constants.schedulerPeriodTime * 10.0;
        m_hoodAngle = Math.max(Math.min(m_hoodAngle, 0), -40);

        int pov = operator.getPOV();

        m_flywheelSpeed +=
            ((pov == 0) ? Constants.schedulerPeriodTime * 2000 : 0) -
            ((pov == 180) ? Constants.schedulerPeriodTime * 2000 : 0);
        m_flywheelSpeed = Math.min(Math.max(m_flywheelSpeed, 0), 6000);

        m_Shooter.setTurretTarget(m_turretAngle, 0);
        m_Shooter.setFlywheelTarget(m_flywheelSpeed);
        m_Shooter.setHoodTarget(m_hoodAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.setFlywheelTarget(0);
        m_Shooter.setHoodTarget(0);
    }
}
