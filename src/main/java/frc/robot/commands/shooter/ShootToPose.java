package frc.robot.commands.shooter;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LED.LEDPattern;
import frc.robot.subsystems.LED.LEDSection.Priority;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.Configuration;
import frc.robot.utils.TrajectorySolver;
import frc.robot.utils.TrajectorySolver.TrajectoryConditions;
import frc.robot.utils.TrajectorySolver.TrajectoryParameters;
import frc.robot.utils.structlogging.StructLogger;

public class ShootToPose extends Command {
    private final Shooter m_Shooter;
    private final Vision m_Vision;
    private final Drive m_Drive;

    private Transform3d m_cameraToTurret;
    private Transform3d m_chassisToTurret;

    private final Supplier<Pose3d> m_targetSupplier;

    private Field2d m_trajectoryDisplay;
    private final int m_displayRes;

    private String m_cameraName;

    private StructLogger m_ballPositionLogger;
    private List<Translation3d> m_ballPositions;
    private List<Translation3d> m_ballVelocities;
    private double m_lastShotTime;

    private StructLogger m_ballVelocityLogger;
    private StructLogger m_targetBallVelocityLogger;
    private StructLogger m_chassisVelocityLogger;

    private final InterpolatingDoubleTreeMap m_treeMap;

    public ShootToPose(Supplier<Pose3d> targetSupplier) {
        super(Shooter.getInstance(), "Targeted Shooting", "ShootToPose");

        m_treeMap = new InterpolatingDoubleTreeMap();
        m_treeMap.put( 0.0, Math.toRadians(65));
        m_treeMap.put( 2.7, Math.toRadians(65));
        m_treeMap.put( 2.71, Math.toRadians(50));
        m_treeMap.put(16.0, Math.toRadians(50));


        m_Shooter = Shooter.getInstance();
        m_Vision = Vision.getInstance();
        m_Drive = Drive.getInstance();
        m_targetSupplier = targetSupplier;

        m_displayRes = 32;

        m_cameraName = "TurretCamera";

        if (RobotBase.isSimulation()) {
            m_ballPositionLogger = StructLogger.translation3dArrayLogger(m_Shooter, "FuelPoses", null);
            m_ballPositions = new ArrayList<>();
            m_ballVelocities = new ArrayList<>();

            m_ballVelocityLogger = StructLogger.translation3dLogger(m_Shooter, "BaseBallVelocity", null);
            m_targetBallVelocityLogger = StructLogger.translation3dLogger(m_Shooter, "TargetBallVelocity", null);
            m_chassisVelocityLogger = StructLogger.translation3dLogger(m_Shooter, "ChassisVelocity", null);
        }

        // Drive/Vision isn't a requirement - it's used for reading only
        addRequirements(m_Shooter);
    }

    @Override 
    public void initialize() {
        Configuration cfg = Configuration.getInstance();
        m_chassisToTurret = new Transform3d(
            new Pose3d(),
            new Pose3d(
                new Translation3d(
                    cfg.getDouble("Shooter", "turretPositionX"),
                    cfg.getDouble("Shooter", "turretPositionY"),
                    cfg.getDouble("Shooter", "turretPositionZ")
                ), Rotation3d.kZero)
        );
    }

    @Override
    public void execute() {
        Pose3d target = m_targetSupplier.get();
        if (target == null) return;

        Pose3d turretPose;
        if (Robot.isSimulation()) {
            Pose3d chassisPose = new Pose3d(m_Drive.getPose());
            turretPose = chassisPose.transformBy(m_chassisToTurret);
        } else {
            turretPose = m_Shooter.getTurretPose();
        }

        double dist = turretPose.getTranslation().toTranslation2d().minus(target.getTranslation().toTranslation2d()).getDistance(Translation2d.kZero);
        double angle = m_treeMap.get(dist);

        ChassisSpeeds chassisSpeeds = m_Drive.getMeasuredSpeeds();
        Translation2d chassisVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        chassisVelocity = chassisVelocity.rotateBy(m_Drive.getPose().getRotation().unaryMinus());

        TrajectoryConditions conditions = new TrajectoryConditions();
        conditions.launch = turretPose.getTranslation();
        conditions.target = target.getTranslation();
        conditions.theta_pitch = angle;
        conditions.chassis_velocity = chassisVelocity;
        
        TrajectoryParameters params = TrajectorySolver.solveTrajectory(conditions);

        double turretYaw = params.theta_yaw;
        double hoodTarget = m_Shooter.pitchToHood(params.theta_pitch);
        double flywheelRPM = m_Shooter.velocityToRPM(params.velocity, params.theta_pitch);

        System.out.printf("Velocity: %f, RPM: %f\n", params.velocity, flywheelRPM);

        m_Shooter.setTurretTarget(turretYaw, 0);
        m_Shooter.setHoodTarget(hoodTarget);
        m_Shooter.setFlywheelTarget(flywheelRPM);

        if (RobotBase.isSimulation()) {
            double simHoodAngle = m_Shooter.hoodToPitch(m_Shooter.getHoodTarget());
            double simFlywheelVelocity = m_Shooter.RPMtoVelocity(m_Shooter.getFlywheelTarget(), simHoodAngle);
            double simTurretTarget = m_Shooter.getTurretTarget();
            Translation3d baseBallVelocity = new Translation3d(simFlywheelVelocity,0,0).rotateBy(new Rotation3d(
                0,
                -simHoodAngle,
                simTurretTarget
            ));

            Translation2d chassisVelocity2d = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
            chassisVelocity2d = chassisVelocity2d.rotateBy(m_Drive.getPose().getRotation().unaryMinus());
            Translation3d targetBallVelocity = baseBallVelocity.plus(new Translation3d(chassisVelocity2d));

            if (Timer.getFPGATimestamp() > m_lastShotTime + 0.2 && m_ballPositions.size() < 64) {
                m_lastShotTime = Timer.getFPGATimestamp();

                m_ballPositions.add(turretPose.getTranslation());
                m_ballVelocities.add(targetBallVelocity);
            }

            for (int i = m_ballPositions.size()-1; i >= 0; i--) {
                if (m_ballPositions.get(i).getZ() < 0) {
                    m_ballPositions.remove(i);
                    m_ballVelocities.remove(i);
                }
            }

            for (int i = 0; i < m_ballPositions.size(); i++) {
                Translation3d ballVelocity = m_ballVelocities.get(i);
                ballVelocity = ballVelocity.plus(new Translation3d(0, 0, -9.8*Constants.schedulerPeriodTime));
                m_ballVelocities.set(i, ballVelocity);

                Translation3d ballPosition = m_ballPositions.get(i);
                ballPosition = ballPosition.plus(ballVelocity.times(Constants.schedulerPeriodTime));
                m_ballPositions.set(i, ballPosition);
            }

            Translation3d[] array = new Translation3d[m_ballPositions.size()];
            m_ballPositions.toArray(array);
            m_ballPositionLogger.setStructArray(array);

            m_ballVelocityLogger.setStruct(baseBallVelocity);
            m_targetBallVelocityLogger.setStruct(targetBallVelocity);
            m_chassisVelocityLogger.setStruct(new Translation3d(chassisVelocity2d));
        }

        //System.out.println(m_Shooter.turretAtTarget() + " " + m_Shooter.flywheelAtTarget() + " " + m_Shooter.hoodAtTarget());
        //if (m_Shooter.turretAtTarget() && m_Shooter.flywheelAtTarget() && m_Shooter.hoodAtTarget()) {
			//LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkGreen, Priority.INFO);
            m_Shooter.chimneySpeed(1);
        //} else {
			//LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkYellow, Priority.INFO);
            //m_Shooter.chimneyStop();
        //}

        m_Shooter.updateTrajectoryDisplay(conditions, params);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.clearTrajectoryDisplay();

        m_Shooter.setFlywheelTarget(0);
        m_Shooter.setHoodTarget(0);
        m_Shooter.chimneyStop();
    }
}
