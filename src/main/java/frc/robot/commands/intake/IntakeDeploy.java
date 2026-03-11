package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;

public class IntakeDeploy extends Command{

    Intake m_intake;
    double m_intakeDeployerSpeed;
    

    public IntakeDeploy(){
        super(Intake.getInstance(), "Intake", "IntakeDeploy");

        m_intake = Intake.getInstance();
        m_intakeDeployerSpeed = Configuration.getInstance().getDouble("Intake", "intakeDeployerSpeed");

        addRequirements(m_intake);

    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {

        m_intake.deploy(m_intakeDeployerSpeed); 
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    
}
