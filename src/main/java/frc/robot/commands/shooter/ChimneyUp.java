package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;

public class ChimneyUp extends Command {
	private final Shooter m_Shooter;

	public ChimneyUp() {
		super(Shooter.getInstance(), "Chimney", "ChimneyUp");
		m_Shooter = Shooter.getInstance();
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		m_Shooter.chimneySpeed(1);
	}
	
	@Override
	public void end(boolean interrupted) {
		m_Shooter.chimneyStop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
