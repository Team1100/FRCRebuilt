package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LED.LEDPattern;
import frc.robot.subsystems.LED.LEDSection.Priority;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.Configuration;
import frc.robot.utils.TrajectorySolver;
import frc.robot.utils.TrajectorySolver.SolveType;
import frc.robot.utils.TrajectorySolver.TrajectoryConditions;
import frc.robot.utils.TrajectorySolver.TrajectoryParameters;

public class ShootToPose extends Command {
    private final Shooter m_Shooter;
    private final Drive m_Drive;
    private final Supplier<Pose3d> m_targetSupplier;

    private Field2d m_trajectoryDisplay;
    private final int m_displayRes;

    public ShootToPose(Supplier<Pose3d> targetSupplier) {
        super(Shooter.getInstance(), "Targeted Shooting", "ShootToPose");

        m_Shooter = Shooter.getInstance();
        m_Drive = Drive.getInstance();
        m_targetSupplier = targetSupplier;

        m_trajectoryDisplay = new Field2d();
        m_trajectoryDisplay.setRobotPose(-10, 0, Rotation2d.kZero);
        new TDSendable(m_Shooter, "Targeted Shooting", "Trajectory Display", m_trajectoryDisplay);

        m_displayRes = 32;

        // Drive isn't a requirement - it's used for reading only
        addRequirements(m_Shooter);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
        Pose3d target = m_targetSupplier.get();
        if (target == null) return;

        Configuration cfg = Configuration.getInstance();
        Pose3d chassisPose = new Pose3d(m_Drive.getPose());
        Pose3d turretPose = new Pose3d(
            cfg.getDouble(m_Shooter.getName(), "turretPositionX"),
            cfg.getDouble(m_Shooter.getName(), "turretPositionY"),
            cfg.getDouble(m_Shooter.getName(), "turretPositionZ"),
            Rotation3d.kZero
        );

        Pose3d trajectoryStart = turretPose.relativeTo(chassisPose);

        TrajectoryConditions conditions = new TrajectoryConditions();
        conditions.start = trajectoryStart;
        conditions.target = target;
        conditions.theta = Math.PI/4;
        
        TrajectoryParameters params = TrajectorySolver.solveTrajectory(conditions, SolveType.CONTROL_THETA);
        
        m_Shooter.setTurretTarget(params.theta_yaw, 0);
        m_Shooter.setHoodTarget(m_Shooter.pitchToHood(params.theta_pitch));
        m_Shooter.setFlywheelTarget(m_Shooter.velocityToRPM(params.velocity));
        if (m_Shooter.turretAtTarget() && m_Shooter.flywheelAtTarget() && m_Shooter.hoodAtTarget()) {
			LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkGreen, Priority.INFO);
            m_Shooter.chimneySpeed(cfg.getDouble("shooter", "chimneyUpSpeed"));
        } else {
			LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkYellow, Priority.INFO);
            m_Shooter.chimneyStop();
        }

        updateTrajectoryDisplay(conditions);
    }

    private void updateTrajectoryDisplay(TrajectoryConditions conditions) {
        for (int i = 0; i < m_displayRes; i++) {
            double t = (conditions.time/(double)m_displayRes)/(i/(double)m_displayRes);
            Pose2d pose = conditions.start.interpolate(conditions.target, t).toPose2d();
            m_trajectoryDisplay.getObject("point" + i).setPose(pose);
        }
    }

    private void clearTrajectoryDisplay() {
        m_trajectoryDisplay.setRobotPose(-10,0,Rotation2d.kZero);
        for (int i = 0; i < m_displayRes; i++) {
            m_trajectoryDisplay.getObject("point" + i).setPose(-10,0,Rotation2d.kZero);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        clearTrajectoryDisplay();
    }
}
