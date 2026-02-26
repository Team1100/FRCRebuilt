package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;

public class IntakeOut extends Command{

    Intake m_intake;
    double m_intakeSpeed;
    

    public IntakeOut(){
        super(Intake.getInstance(), "Intake", "IntakeOut");

        m_intake = Intake.getInstance();
        m_intakeSpeed = Configuration.getInstance().getDouble("Intake", "intakeSpeed");

        addRequirements(m_intake);

    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {

        m_intake.spinOut(m_intakeSpeed); 
        //TODO use same variable as IntakeIn or vice versa 
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
