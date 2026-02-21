package frc.robot.commands.global;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;

public class Unjam extends Command{

    Intake m_intake;
    Spindexer m_spindexer;
    Shooter m_shooter;

    double m_intakeSpeed; //i should have named the variable intakeRollerSpeed my bad
    double m_intakeDeployerSpeed; 
    double m_spindexerSpeed;
    double m_chimneySpeed;
    double m_flywheelSpeed;
    
    public Unjam(){
        super(Intake.getInstance(), "Intake", "Unjam"); //TODO: please allow for not one specific subsystem

        m_intake = Intake.getInstance();
        m_intakeSpeed = Configuration.getInstance().getDouble("Intake", "intakeSpeed");
        m_intakeDeployerSpeed = Configuration.getInstance().getDouble("Intake", "intakeDeployerSpeed");

        m_spindexer = Spindexer.getInstance();
        m_spindexerSpeed = Configuration.getInstance().getDouble("Spindexer", "spindexerSpeed");

        m_shooter = Shooter.getInstance();
        m_chimneySpeed = Configuration.getInstance().getDouble("Chimney", "chimneySpeed");
        m_flywheelSpeed = Configuration.getInstance().getDouble("Shooter", "flywheelVelocityFactor"); //right variable?
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {

        m_intake.spinOut(m_intakeSpeed);
        m_intake.deploy(m_intakeDeployerSpeed);
        m_spindexer.spinOut(m_spindexerSpeed);
        m_shooter.chimneySpeed(m_chimneySpeed * -1); //no method for reverse so gotta do it here
        m_shooter.setFlywheelTarget(m_flywheelSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_spindexer.stop();
        m_shooter.chimneyStop();
        m_shooter.setFlywheelTarget(m_flywheelSpeed);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
