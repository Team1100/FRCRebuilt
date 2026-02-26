package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;

public class IntakeIn extends Command{
    private final Intake m_intake;
    private final double m_intakeSpeed;

    public IntakeIn(){
        super(Intake.getInstance(), "Intake", "IntakeIn");

        m_intake = Intake.getInstance();
        m_intakeSpeed = Configuration.getInstance().getDouble("Intake", "rollerSpeed");
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
        m_intake.spinIn(m_intakeSpeed); 
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
