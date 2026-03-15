package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;

public class IntakeAAAAA extends Command {
    private final Intake m_intake;
    private final double m_intakeSpeed;

    public IntakeAAAAA() {
        super(Intake.getInstance(), "Intake", "IntakeAAAAA");

        m_intake = Intake.getInstance();
        m_intakeSpeed = 1.0;

        addRequirements(m_intake);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
        m_intake.spinOut(m_intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    
}
