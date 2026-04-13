package frc.robot.commands.shooter;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LED.LEDPattern;
import frc.robot.subsystems.LED.LEDSection.Priority;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.structlogging.StructLogger;
import frc.robot.utils.trajectory.TrajectorySolver;
import frc.robot.utils.trajectory.VelocityMapping;
import frc.robot.utils.trajectory.TrajectorySolver.TrajectoryConditions;
import frc.robot.utils.trajectory.TrajectorySolver.TrajectoryParameters;

public class ShootToPose extends Command {
    private final Shooter m_Shooter;
    private final Drive m_Drive;

    private Transform3d m_chassisToTurret;

    private final Supplier<Pose3d> m_targetSupplier;

    private StructLogger m_ballPositionLogger;
    private List<Translation3d> m_ballPositions;

    private StructLogger m_ballVelocityLogger;
    private StructLogger m_correctedTargetLogger;
    private StructLogger m_chassisVelocityLogger;

    private final InterpolatingDoubleTreeMap m_distAngleMap;

    private final VelocityMapping m_overrideVMap;

    private static final Map<Double, Double> DEFAULT_ANGLEMAP = Map.of(
        0.0, Math.toRadians(65),
        2.7, Math.toRadians(65),
        2.71, Math.toRadians(60),
        16.0, Math.toRadians(60)
    );

    private static final int ITERATIONS = 20;
    private static final double ERROR_THRESHOLD = 1e-2;

    private static final int DISPLAY_RES = 16;

    public ShootToPose(Supplier<Pose3d> targetSupplier) {
        this(targetSupplier, DEFAULT_ANGLEMAP, null);
    }

    public ShootToPose(Supplier<Pose3d> targetSupplier, Map<Double, Double> distAngleMap, VelocityMapping overrideVMap) {
        super(Shooter.getInstance(), "Targeted Shooting", "ShootToPose");

        m_Shooter = Shooter.getInstance();
        m_Drive = Drive.getInstance();
        m_targetSupplier = targetSupplier;

        m_overrideVMap = overrideVMap;

        m_distAngleMap = new InterpolatingDoubleTreeMap();
        for (Double key : distAngleMap.keySet()) {
            m_distAngleMap.put(key, distAngleMap.get(key));
        }

        if (RobotBase.isSimulation()) {
            m_ballPositionLogger = StructLogger.translation3dArrayLogger(m_Shooter, "FuelPoses", null);
            m_ballPositions = new ArrayList<>();

            m_ballVelocityLogger = StructLogger.translation3dLogger(m_Shooter, "BaseBallVelocity", null);
            m_correctedTargetLogger = StructLogger.translation3dLogger(m_Shooter, "TargetBallVelocity", null);
            m_chassisVelocityLogger = StructLogger.translation3dLogger(m_Shooter, "ChassisVelocity", null);
        }


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

        // Drive/Vision isn't a requirement - it's used for reading only
        addRequirements(m_Shooter);
    }

    @Override 
    public void initialize() {}

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

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_Drive.getMeasuredSpeeds(), m_Drive.getPose().getRotation());

        TrajectoryConditions conditions = new TrajectoryConditions();
        TrajectoryParameters params = null;

        for (int i = 0; i < ITERATIONS; i++) {
            Translation3d compensatingTarget;
            if (params != null) {
                Twist2d twist = m_Drive.getFieldRelativeTwist(params.time);//chassisSpeeds.toTwist2d(params.time);
                compensatingTarget = target.exp(new Twist3d(twist.dx, twist.dy, 0, 0, 0, twist.dtheta)).getTranslation();
            } else {
                compensatingTarget = target.getTranslation();
            }

            double dist = turretPose.getTranslation().toTranslation2d().getDistance(compensatingTarget.toTranslation2d());
            double angle = m_distAngleMap.get(dist);
            
            conditions.launch = turretPose.getTranslation();
            conditions.target = compensatingTarget;
            conditions.theta_pitch = angle;
            conditions.compensate_for_velocity = false;
            
            TrajectoryParameters newParams = TrajectorySolver.solveTrajectory(conditions);
            
            if (Double.isNaN(newParams.time)) break;

            double error = Double.MAX_VALUE;
            if (params != null) {
                error = (params.time-newParams.time)/(newParams.time)
                      + (params.theta_pitch-newParams.theta_pitch)/(newParams.theta_pitch)
                      + (params.theta_yaw-newParams.theta_yaw)/(newParams.theta_yaw);
            }
            System.out.println("Iteration: " + i + "; Error: " + error);
            params = newParams;

            if (Math.abs(error) < ERROR_THRESHOLD) break;
        }

        if (params == null) return;

        double turretYaw = params.theta_yaw;
        double hoodTarget = m_Shooter.pitchToHood(params.theta_pitch);
        double flywheelRPM = m_Shooter.velocityToRPM(params.velocity, params.theta_pitch, m_overrideVMap);

        System.out.printf("Velocity: %f, RPM: %f\n", params.velocity, flywheelRPM);

        m_Shooter.setTurretTarget(turretYaw, 0);
        m_Shooter.setHoodTarget(hoodTarget);
        m_Shooter.setFlywheelTarget(flywheelRPM);

        // System.out.println(m_Shooter.turretAtTarget() + " " + m_Shooter.flywheelAtTarget() + " " + m_Shooter.hoodAtTarget());
        if (m_Shooter.flywheelAtTarget() && m_Shooter.hoodAtTarget()) {
            // LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkGreen, Priority.INFO);
            m_Shooter.chimneySpeed(1);
        } else {
            // LED.getInstance().setPattern(1, LEDPattern.kCheckeredBlinkYellow, Priority.INFO);
            m_Shooter.chimneyStop();
        }

        if (RobotBase.isSimulation()) {
            Translation3d baseBallVelocity = new Translation3d(params.velocity,0,0).rotateBy(new Rotation3d(
                0,
                -params.theta_pitch,
                params.theta_yaw
            ));

            Translation3d correctedTarget = conditions.target;

            m_ballPositions.clear();
            for (int i = 0; i < DISPLAY_RES; i++) {
                double t = params.time*((double)i/(DISPLAY_RES-1));

                Translation3d targetBallOffset = baseBallVelocity
                    .times(t)
                    .plus(new Translation3d(0, 0, -4.9*t*t));
                m_ballPositions.add(conditions.launch.plus(targetBallOffset));

                // Translation3d physBallOffset = baseBallVelocity
                //     .plus(new Translation3d(chassisVelocity))
                //     .plus(new Translation3d(angLinVel))
                //     .times(t)
                //     .plus(new Translation3d(0, 0, -4.9*t*t));
                // m_ballPositions.add(conditions.launch.plus(physBallOffset));
            }

            Translation3d[] array = new Translation3d[m_ballPositions.size()];
            m_ballPositions.toArray(array);
            m_ballPositionLogger.setStructArray(array);

            m_ballVelocityLogger.setStruct(baseBallVelocity);
            m_correctedTargetLogger.setStruct(correctedTarget);
            // m_chassisVelocityLogger.setStruct(new Translation3d(chassisVelocity));
        }

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

    public static ShootToPose withFixedValues(Supplier<Pose3d> targetSupplier, double angle, VelocityMapping overrideVelocityMapping) {
        return new ShootToPose(targetSupplier, Map.of(0.0,angle), overrideVelocityMapping);
    }

    public static ShootToPose hubTargetting() {
        return new ShootToPose(FieldUtils.getInstance()::getHubPose, DEFAULT_ANGLEMAP, null);
    }
}
