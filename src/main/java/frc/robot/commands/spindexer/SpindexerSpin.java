package frc.robot.commands.spindexer;

import frc.robot.subsystems.Spindexer;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;

public class SpindexerSpin extends Command{

    Spindexer m_spindexer;
    double m_spindexerSpeed;
    

    public SpindexerSpin(){
        super(Spindexer.getInstance(), "Spindexer", "SpindexerSpin");

        m_spindexer = Spindexer.getInstance();
        m_spindexerSpeed = Configuration.getInstance().getDouble("Spindex", "spindexSpeed");

        addRequirements(m_spindexer);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
        
        m_spindexer.spinIn(m_spindexerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_spindexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
