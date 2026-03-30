package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.SwerveTurretPoseEstimator3d;
import frc.robot.utils.TrajectorySolver.TrajectoryConditions;
import frc.robot.utils.TrajectorySolver.TrajectoryParameters;
import frc.robot.utils.sensing.SparkCurrentLimitDetector;
import frc.robot.utils.sensing.SparkCurrentLimitDetector.HardLimitDirection;
import frc.robot.utils.vision.VisionEstimationResult;

public class Shooter extends SubsystemBase {
    private static Shooter m_Shooter = null;

    // Flywheel
    private final boolean m_flywheelEnabled;
    private SparkBase m_flywheelLeftMotor;
    private SparkBase m_flywheelRightMotor;

    private SparkBaseConfig m_flywheelLeftConfig;

    private double m_flywheelP;
    private double m_flywheelI;
    private double m_flywheelD;
    private TDNumber m_TDflywheelP;
    private TDNumber m_TDflywheelI;
    private TDNumber m_TDflywheelD;

    private TDNumber m_TDflywheelKs;
    private TDNumber m_TDflywheelKv;
    private TDNumber m_TDflywheelKa;

    private boolean m_tuneFlywheel;

    private SimpleMotorFeedforward m_flywheelFF;

    // private TDNumber m_TDflywheelLinTermM;
    // private TDNumber m_TDflywheelLinTermB;
    private InterpolatingDoubleTreeMap m_flywheelVelocityTermsM;
    private InterpolatingDoubleTreeMap m_flywheelVelocityTermsB;

    private TDNumber m_TDflywheelVelocity;
    private TDNumber m_TDflywheelMeasuredVelocity;
    private TDNumber m_TDflywheelMeasuredCurrent;

    // Turret
    private final boolean m_turretEnabled;

    private boolean m_tuneTurret;

    private SparkBase m_turretMotor;

    private SparkBaseConfig m_turretConfig;

    private SparkCurrentLimitDetector m_turretCurrentLimit;
    private double m_turretForwardHardLimit;
    private double m_turretReverseHardLimit;
    private double m_turretForwardSoftLimit;
    private double m_turretReverseSoftLimit;

    private boolean m_turretCalibrationEnabled;
    public enum TurretCalibration {
        CALIBRATE_FORWARD,
        CALIBRATE_FULL
    }
    private TurretCalibration m_turretCalibration;
    private boolean m_turretCalibratedForward;

    private double m_turretP;
    private double m_turretI;
    private double m_turretD;
    private TDNumber m_TDturretP;
    private TDNumber m_TDturretI;
    private TDNumber m_TDturretD;

    private TDNumber m_TDturretKs;
    private TDNumber m_TDturretKv;
    private TDNumber m_TDturretKa;
    private TrapezoidProfile m_turretProfile;
    private TrapezoidProfile.State m_turretState;
    private TrapezoidProfile.State m_turretSetpoint;
    private SimpleMotorFeedforward m_turretFF;

    private double m_turretTolerance;

    private TDNumber m_TDturretTargetAngle;
    private TDNumber m_TDturretSpeed;

    private TDNumber m_TDturretMeasuredPosition;
    private TDNumber m_TDturretMeasuredVelocity;
    private TDNumber m_TDturretProfilePosition;
    private TDNumber m_TDturretMeasuredCurrent;

    SwerveTurretPoseEstimator3d m_turretPoseEstimator;
    Field2d m_turretPoseField;
    private Pose3d m_turretPose;
    private boolean m_turretGotResult;

    private Field2d m_trajectoryDisplay;

    private boolean m_turretRobotRelative;
    private boolean m_turretControl;

    // Hood
    private final boolean m_hoodEnabled;

    private boolean m_tuneHood;

    private SparkBase m_hoodMotor;
    private SparkBaseConfig m_hoodConfig;

    private RelativeEncoder m_hoodEncoder;

    private double m_hoodKg;
    private double m_hoodKs;
    private double m_hoodKv;
    private double m_hoodP;
    private double m_hoodI;
    private double m_hoodD;
    private TDNumber m_TDhoodKg;
    private TDNumber m_TDhoodKs;
    private TDNumber m_TDhoodKv;
    private TDNumber m_TDhoodP;
    private TDNumber m_TDhoodI;
    private TDNumber m_TDhoodD;

    private SparkClosedLoopController m_hoodClosedLoopController;
    private ElevatorFeedforward m_hoodFF;
    private TrapezoidProfile m_hoodProfile;
    private TrapezoidProfile.State m_hoodState;
    private TrapezoidProfile.State m_hoodSetpoint;
    private double m_hoodTolerance;

    private SparkCurrentLimitDetector m_hoodLimiter;

    private TDNumber m_TDHoodMotorCurrent;
    private TDNumber m_TDHoodPosition;
    private TDNumber m_TDHoodProfilePosition;
    private TDNumber m_TDhoodTargetPosition;

    private Drive m_Drive;
    private Vision m_Vision;

    // Chimney
    private final boolean m_chimneyEnabled;
    private SparkBase m_chimneyMotor;
    private SparkBaseConfig m_chimneyConfig;

    private TDNumber m_TDchimneyMeasuredCurrent;

    private Shooter() {
        super("Shooter");

        m_flywheelEnabled = cfgBool("flywheelEnabled");
        m_turretEnabled = cfgBool("turretEnabled");
        m_hoodEnabled = cfgBool("hoodEnabled");
        m_chimneyEnabled = cfgBool("chimneyEnabled");

        if (m_flywheelEnabled)
            setupFlywheel();

        if (m_turretEnabled)
            setupTurret();

        if (m_hoodEnabled)
            setupHood();

        if (m_chimneyEnabled)
            setupChimney();
    }

    public static Shooter getInstance() {
        if (m_Shooter == null) {
            m_Shooter = new Shooter();
        }
        return m_Shooter;
    }

    private void setupFlywheel() {
        m_tuneFlywheel = cfgBool("tuneFlywheel");

        var flywheelLeftMotorConfig = config().getMotorController("flywheel1");
        var flywheelRightMotorConfig = config().getMotorController("flywheel2");

        m_flywheelLeftMotor = flywheelLeftMotorConfig.m_controller;
        m_flywheelRightMotor = flywheelRightMotorConfig.m_controller;

        m_TDflywheelP = new TDNumber(this, "Flywheel", "P");
        m_TDflywheelI = new TDNumber(this, "Flywheel", "I");
        m_TDflywheelD = new TDNumber(this, "Flywheel", "D");
        m_TDflywheelP.set(cfgDbl("flywheelP"));
        m_TDflywheelI.set(cfgDbl("flywheelI"));
        m_TDflywheelD.set(cfgDbl("flywheelD"));
        m_flywheelP = m_TDflywheelP.get();
        m_flywheelI = m_TDflywheelI.get();
        m_flywheelD = m_TDflywheelD.get();

        m_TDflywheelKs = new TDNumber(this, "Flywheel", "Ks");
        m_TDflywheelKv = new TDNumber(this, "Flywheel", "Kv");
        m_TDflywheelKa = new TDNumber(this, "Flywheel", "Ka");
        m_TDflywheelKs.set(cfgDbl("flywheelKs"));
        m_TDflywheelKv.set(cfgDbl("flywheelKv"));
        m_TDflywheelKa.set(cfgDbl("flywheelKa"));

        m_flywheelLeftConfig = flywheelLeftMotorConfig.m_config;
        m_flywheelLeftConfig.closedLoop.pid(m_flywheelP, m_flywheelI, m_flywheelD);
        m_flywheelLeftConfig.encoder
            .velocityConversionFactor(cfgDbl("flywheelVelocityFactor"))
            .quadratureAverageDepth(8)
            .quadratureMeasurementPeriod(25);

        m_flywheelLeftMotor.configure(m_flywheelLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig flywheelRightConfig = flywheelRightMotorConfig.m_config;
        flywheelRightConfig.follow(m_flywheelLeftMotor, true);
        m_flywheelRightMotor.configure(flywheelRightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_flywheelFF = new SimpleMotorFeedforward(
                m_TDflywheelKs.get(), m_TDflywheelKv.get(), m_TDflywheelKa.get());

        m_turretTolerance = cfgDbl("turretTolerance");

        m_TDflywheelVelocity = new TDNumber(this, "Flywheel", "Target Velocity");
        m_TDflywheelVelocity.set(0);

        // m_TDflywheelLinTermM = new TDNumber(this, "VelocityMapping", "TermM", true);
        // m_TDflywheelLinTermM.set(ShooterConstants.kVelocityLinTermM_45);
        // m_TDflywheelLinTermB = new TDNumber(this, "VelocityMapping", "TermB", true);
        // m_TDflywheelLinTermB.set(ShooterConstants.kVelocityLinTermB_45);

        m_flywheelVelocityTermsM = new InterpolatingDoubleTreeMap();
        m_flywheelVelocityTermsB = new InterpolatingDoubleTreeMap();
        for (double[] mapping : ShooterConstants.kVelocityLinTerms) {
            m_flywheelVelocityTermsM.put(mapping[0], mapping[1]);
            m_flywheelVelocityTermsB.put(mapping[0], mapping[2]);
        }

        m_TDflywheelMeasuredVelocity = new TDNumber(this, "Flywheel", "Measured Velocity");
        m_TDflywheelMeasuredCurrent = new TDNumber(this, "Flywheel", "Measured Current");
    }

    private void setupTurret() {
        m_tuneTurret = cfgBool("tuneTurret");

        var turretMotorConfig = config().getMotorController("turretRotation");
        m_turretMotor = turretMotorConfig.m_controller;

        m_TDturretP = new TDNumber(this, "Turret", "P");
        m_TDturretI = new TDNumber(this, "Turret", "I");
        m_TDturretD = new TDNumber(this, "Turret", "D");

        m_TDturretP.set(cfgDbl("turretP"));
        m_TDturretI.set(cfgDbl("turretI"));
        m_TDturretD.set(cfgDbl("turretD"));
        m_turretP = m_TDturretP.get();
        m_turretI = m_TDturretI.get();
        m_turretD = m_TDturretD.get();

        m_turretConfig = turretMotorConfig.m_config;
        m_turretConfig.inverted(true);
        m_turretConfig.idleMode(IdleMode.kBrake);
        m_turretConfig.smartCurrentLimit(cfgInt("turretStallCurrentLimit"), cfgInt("turretFreeCurrentLimit"));
        m_turretConfig.encoder
            .positionConversionFactor(Constants.ShooterConstants.kTurretPositionFactor)
            .velocityConversionFactor(Constants.ShooterConstants.kTurretPositionFactor/60);
        m_turretConfig.closedLoop.pid(m_turretP, m_turretI, m_turretD);
        m_turretConfig.closedLoop.positionWrappingEnabled(false);

        m_turretMotor.configure(m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_turretCurrentLimit = new SparkCurrentLimitDetector(m_turretMotor, cfgDbl("turretCurrentTrip"), cfgDbl("turretZeroSpeedTolerance"));
        m_turretForwardHardLimit = cfgDbl("turretForwardHardLimit");
        m_turretReverseHardLimit = cfgDbl("turretReverseHardLimit");
        m_turretForwardSoftLimit = m_turretForwardHardLimit - cfgDbl("turretSoftLimitWidth");
        m_turretReverseSoftLimit = m_turretReverseHardLimit + cfgDbl("turretSoftLimitWidth");

        m_turretCalibrationEnabled = false;

        Constraints constraints = new Constraints(Constants.ShooterConstants.kTurretMaxVelocity,
                Constants.ShooterConstants.kTurretMaxAccel);
        m_turretProfile = new TrapezoidProfile(constraints);

        m_TDturretKs = new TDNumber(this, "Turret", "Ks");
        m_TDturretKv = new TDNumber(this, "Turret", "Kv");
        m_TDturretKa = new TDNumber(this, "Turret", "Ka");

        m_TDturretKs.set(cfgDbl("turretKs"));
        m_TDturretKv.set(cfgDbl("turretKv"));
        m_TDturretKa.set(cfgDbl("turretKa"));

        m_turretFF = new SimpleMotorFeedforward(m_TDturretKs.get(), m_TDturretKv.get(), m_TDturretKa.get());

        m_TDturretTargetAngle = new TDNumber(this, "Turret", "Target Angle");
        m_TDturretSpeed = new TDNumber(this, "Turret", "Turret Speed");

        m_TDturretMeasuredPosition = new TDNumber(this, "Turret", "Measured Position");
        m_TDturretMeasuredVelocity = new TDNumber(this, "Turret", "Measured Velocity");
        m_TDturretMeasuredCurrent = new TDNumber(this, "Turret", "Measured Current");
        m_TDturretProfilePosition = new TDNumber(this, "Turret", "Profile Position");

        m_Drive = Drive.getInstance();
        
        m_turretPose = new Pose3d(m_Drive.getPose()).plus(new Transform3d(new Pose3d(), new Pose3d(
            new Translation3d(
                cfgDbl("turretPositionX"),
                cfgDbl("turretPositionY"),
                cfgDbl("turretPositionZ")),
            Rotation3d.kZero
        )));
        m_turretGotResult = false;

        double initPosition = 0;
        m_turretSetpoint = new TrapezoidProfile.State(initPosition, 0.0);
        m_turretState = new TrapezoidProfile.State(initPosition, 0.0);

        m_turretRobotRelative = cfgBool("turretRobotRelative");
        m_turretControl = true;
        m_Vision = Vision.getInstance();

        var turretVision = m_Vision.getVisionSystemByName(Constants.ShooterConstants.kTurretCameraName);
        if(turretVision != null) {
            turretVision.setExpectedHeight(cfgDbl("turretPositionZ"));
        } else {
            System.out.println("WARNING: Turret Vision System does not exist, turret localization may not work");
        }

        m_turretPoseEstimator = new SwerveTurretPoseEstimator3d(
            Constants.DriveConstants.kinematics, 
            new Rotation3d(0, 0, Math.toRadians(m_Drive.getGyroAngle())),
            Rotation2d.k180deg,
            new Rotation2d(m_turretMotor.getEncoder().getPosition()),
            m_Drive.getModulePositions(), 
            new Pose3d(),
            new Transform3d(cfgDbl("turretPositionX"), cfgDbl("turretPositionY"), cfgDbl("turretPositionZ"), Rotation3d.kZero));

        m_turretPoseField = new Field2d();
        new TDSendable(this, "Field", "Turret Position", m_turretPoseField);

        m_trajectoryDisplay = new Field2d();
        m_trajectoryDisplay.setRobotPose(-10, 0, Rotation2d.kZero);
        new TDSendable(this, "Field", "Trajectory Display", m_trajectoryDisplay);
    }

    private void setupHood() {
        m_tuneHood = cfgBool("tuneHood");

        var hoodMotorConfig = config().getMotorController("hood");
        
        m_hoodMotor = hoodMotorConfig.m_controller;

        m_TDhoodKg = new TDNumber(this, "Hood", "Kg");
        m_TDhoodKs = new TDNumber(this, "Hood", "Ks");
        m_TDhoodKv = new TDNumber(this, "Hood", "Kv");
        m_TDhoodP = new TDNumber(this, "Hood", "P");
        m_TDhoodI = new TDNumber(this, "Hood", "I");
        m_TDhoodD = new TDNumber(this, "Hood", "D");

        m_TDhoodKg.set(cfgDbl("hoodKg"));
        m_TDhoodKs.set(cfgDbl("hoodKs"));
        m_TDhoodKv.set(cfgDbl("hoodKv"));
        m_TDhoodP.set(cfgDbl("hoodP"));
        m_TDhoodI.set(cfgDbl("hoodI"));
        m_TDhoodD.set(cfgDbl("hoodD"));

        m_hoodKg = m_TDhoodKg.get();
        m_hoodKs = m_TDhoodKs.get();
        m_hoodKv = m_TDhoodKv.get();
        m_hoodP = m_TDhoodP.get();
        m_hoodI = m_TDhoodI.get();
        m_hoodD = m_TDhoodD.get();

        m_hoodConfig = hoodMotorConfig.m_config;
        m_hoodConfig.idleMode(IdleMode.kBrake);
        m_hoodConfig.closedLoop
                .pid(m_hoodP, m_hoodI, m_hoodD)
                .positionWrappingEnabled(false);
        m_hoodConfig.encoder
                .positionConversionFactor(Constants.ShooterConstants.kHoodPositionConversion)
                .velocityConversionFactor(Constants.ShooterConstants.kHoodVelocityConversion);

        m_hoodMotor.configure(m_hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_hoodEncoder = m_hoodMotor.getEncoder();
        m_hoodClosedLoopController = m_hoodMotor.getClosedLoopController();

        m_hoodLimiter = new SparkCurrentLimitDetector(
                m_hoodMotor,
                Constants.ShooterConstants.kHoodStallDetectCurrent,
                Constants.ShooterConstants.kHoodStallSpeedTolerance);

        TrapezoidProfile.Constraints hoodConstraints = new TrapezoidProfile.Constraints(
                Constants.ShooterConstants.kHoodMaxVelocityDegreesPerSec,
                Constants.ShooterConstants.kHoodMaxAcceleration);
        m_hoodProfile = new TrapezoidProfile(hoodConstraints);
        m_hoodState = new TrapezoidProfile.State(m_hoodEncoder.getPosition(), 0.0);
        m_hoodSetpoint = new TrapezoidProfile.State(m_hoodEncoder.getPosition(), 0.0);
        m_hoodFF = new ElevatorFeedforward(m_hoodKs, m_hoodKg, m_hoodKv);

        m_TDHoodMotorCurrent = new TDNumber(this, "Hood", "Motor Current");
        m_TDHoodPosition = new TDNumber(this, "Hood", "Position");
        m_TDHoodProfilePosition = new TDNumber(this, "Hood", "Profile Position");
        m_TDhoodTargetPosition = new TDNumber(this, "Hood", "Hood Target Position");

        m_hoodTolerance = cfgDbl("hoodTolerance");
    }

    private void setupChimney() {
        var chimneyMotorConfig = config().getMotorController("chimney");
        m_chimneyMotor = chimneyMotorConfig.m_controller;
        m_chimneyConfig = chimneyMotorConfig.m_config;
        m_chimneyConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(0, 0);

        m_chimneyMotor.configure(m_chimneyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_TDchimneyMeasuredCurrent = new TDNumber(this, "Chimney", "Measured Current");
    }

    /**
     * Sets a target velocity for the flywheel.
     * 
     * @param rpm
     *            Velocity to approach in RPM.
     */
    public void setFlywheelTarget(double rpm) {
        if (!m_flywheelEnabled) return;

        m_TDflywheelVelocity.set(rpm);
    }

    /**
     * Sets a target position for the hood.
     * 
     * @param angle
     *            Position to approach in radians.
     */
    public void setHoodTarget(double angle) {
        if (!m_hoodEnabled) return;

        double clampedAngle = MathUtil.clamp(angle,
                Constants.ShooterConstants.kHoodMinAngle,
                Constants.ShooterConstants.kHoodMaxAngle);
        m_hoodSetpoint = new TrapezoidProfile.State(clampedAngle, 0.0);
        m_TDhoodTargetPosition.set(clampedAngle);
    }

    /**
     * Sets the target values of the turret.
     * 
     * @param targetAngle
     *            Target angle to approach in radians.
     * @param speed
     *            Speed to move target angle by in radians/second.
     */
    public void setTurretTarget(double targetAngle, double speed) {
        if (!m_turretEnabled) return;

        m_TDturretTargetAngle.set(targetAngle);
        m_TDturretSpeed.set(speed);
    }

    public void chimneySpeed(double speed) {
        if (m_chimneyMotor != null)
            m_chimneyMotor.set(speed);
    }

    public void chimneyStop() {
        if (m_chimneyMotor != null)
            m_chimneyMotor.set(0);
    }

    public double getTurretTarget() {
        return m_TDturretTargetAngle.get();
    }

    public double getTurretSpeed() {
        return m_TDturretSpeed.get();
    }

    public boolean turretAtTarget() {
        return MathUtil.isNear(m_TDturretTargetAngle.get(), m_turretMotor.getEncoder().getPosition(),
                m_turretTolerance);
    }

    public double getFlywheelTarget() {
        return m_TDflywheelVelocity.get();
    }

    public boolean flywheelAtTarget() {
        return MathUtil.isNear(m_TDflywheelVelocity.get(), m_turretMotor.getEncoder().getVelocity(),
                cfgDbl("flywheelTolerance"));
    }

    public double getHoodTarget() {
        return m_TDhoodTargetPosition.get();
    }

    public void resetTurretEstimatorPose(Pose3d newPose) {
        double angle = Math.toRadians(m_Drive.getGyroAngle());
        m_turretPoseEstimator.resetPosition(
            new Rotation3d(0,0,angle),
            m_Drive.getModulePositions(),
            newPose);
    }

    public void resetTurretEstimatorRotation(double gyroAngleDegs) {
        m_turretPoseEstimator.resetRotation(new Rotation3d(0, 0, Math.toRadians(gyroAngleDegs)));
    }

    public boolean hoodAtTarget() {
        return MathUtil.isNear(m_TDhoodTargetPosition.get(), m_hoodMotor.getEncoder().getPosition(), m_hoodTolerance);
    }

    public void updateTrajectoryDisplay(TrajectoryConditions conditions, TrajectoryParameters params) {
        int res = 32;
        for (int i = 0; i < res; i++) {
            double t = i/(double)(res-1);
            Translation3d inter = conditions.launch
                .interpolate(conditions.target, t);
            Pose2d pose = new Pose2d(inter.toTranslation2d(), Rotation2d.kZero);
            m_trajectoryDisplay.getObject("point" + i).setPose(pose);
        }
    }

    public void clearTrajectoryDisplay() {
        int res = 32;
        m_trajectoryDisplay.setRobotPose(-10,0,Rotation2d.kZero);
        for (int i = 0; i < res; i++) {
            m_trajectoryDisplay.getObject("point" + i).setPose(-10,0,Rotation2d.kZero);
        }
    }

    /**
     * Gets a hood angle needed to throw the ball at a given pitch
     * 
     * @param angle
     *            Input pitch in radians (0 = +X, pi/2 = +Z)
     * @return Valid hood angle in degrees
     */
    public double pitchToHood(double angle) {
        double tranlatedAngle = Math.PI/2 - angle;
        double degs = Math.toDegrees(-tranlatedAngle) + Constants.ShooterConstants.kHoodAngleOffset;
        return MathUtil.clamp(degs, Constants.ShooterConstants.kHoodMinAngle, Constants.ShooterConstants.kHoodMaxAngle);
    }

    public double hoodToPitch(double hood) {
        // deg = rtod(-pi/2 + angle) + offset
        // deg - offset = rtod(-pi/2 + angle)
        // dtor(deg - offset) = -pi/2 + angle
        // dtor(deg - offset) + pi/2 = angle
        return Math.toRadians(hood - Constants.ShooterConstants.kHoodAngleOffset) + Math.PI/2;
    }

    /**
     * Convert a field velocity to a flywheel velocity for the flywheel
     * 
     * @param velocity
     *            Field velocity in m/s
     * @return Flywheel velocity in RPM
     */
    public double velocityToRPM(double velocity, double angle) {
        if (ShooterConstants.kVelocityMapQuadratic) {
            final double a = ShooterConstants.kVelocityQuadTermA;
            final double b = ShooterConstants.kVelocityQuadTermB;
            final double c = ShooterConstants.kVelocityQuadTermC;

            final double rpm = (-b + Math.sqrt(b*b - 4*a*(c-velocity)))/(2*a);
            return Math.max(rpm, 0);
        } else {
            final double m = m_flywheelVelocityTermsM.get(angle);
            final double b = m_flywheelVelocityTermsB.get(angle);

            final double rpm = (velocity - b)/m;
            return Math.max(rpm, 0);
        }
    }

    public double RPMtoVelocity(double rpm, double angle) {
        if (ShooterConstants.kVelocityMapQuadratic) {
            final double a = ShooterConstants.kVelocityQuadTermA;
            final double b = ShooterConstants.kVelocityQuadTermB;
            final double c = ShooterConstants.kVelocityQuadTermC;

            return a*rpm*rpm + b*rpm + c;
        } else {
            final double m = m_flywheelVelocityTermsM.get(angle);
            final double b = m_flywheelVelocityTermsB.get(angle);
            
            return m*rpm + b;
        }
    }

    // see angleToTarget for more info
    enum TurretState {
        SHOOTING, // direct movement, take the shortest available movement
        FERRYING, // indirect movement, try to get turret into the 'free range' closest to hub
        ROBOT_RELATIVE
    }

    /**
     * Get an angle relative to the field and transform it into an angle for
     * the turret to move to.
     * 
     * There's two different ways to optimize the turret's rotation:
     * first, move to the closest angle, and wrap at hard limits;
     * second, reduce interaction with the hard limits by staying in the 'free
     * range' closest
     * to the hub.
     * 
     * <br>
     * </br>
     * 
     * <b>TurretState.SHOOTING</b> refers to the first optimization mode.
     * it simply calculates the shortest movement to the target angle, and if that
     * passes
     * over the hard limit, then it instead wraps around over the hard limit,
     * causing a
     * 360 degree rotation.
     * 
     * <br>
     * </br>
     * 
     * <b>TurretState.FERRYING</b> refers to the second optimization mode.
     * While the robot is ferrying, it'll follow a 'snaking' pattern, thus spending
     * most
     * of the time with robot +X parallel to field Y-axis. This means there's a
     * certain
     * 180 degree range which doesn't touch the hard limits, called the 'free
     * range'.
     * When robot +X is parallel to field Y-axis, having the turret be in the
     * hub-side
     * free range means it'll never have to cross over a limit, thus maintaining
     * perfect
     * shortest-distance motions.
     * 
     * <br>
     * </br>
     * 
     * That's a lot of words for: always follow the version of the target angle that
     * lies in the
     * range -180 to 180. When the turret is already in its hub-side free range,
     * this will
     * result in the shortest distance. When the turret is on the corner, it'll
     * invert
     * partway through the turn, but this turret should be quick enough to finish
     * before
     * the corner chassis turning is even finished.
     * 
     * @param targetAngle
     *            Target angle in field orientation.
     * @param state
     *            Type of motion to optimize for.
     * @return Optimized target angle in robot orientation.
     */
    public double angleToTarget(double targetAngle, TurretState state) {
        // current target position, robot oriented
        double robotAngle = m_turretMotor.getEncoder().getPosition();

        Pose2d chassisPose = m_Drive.getPose();
        Rotation2d chassisRotation = chassisPose.getRotation().plus(Rotation2d.k180deg);
        double chassisAngle = chassisRotation.getRadians();
        // current target angle, robot oriented
        double robotTargetAngle = MathUtil.angleModulus(targetAngle - chassisAngle);

        switch (state) {
            case SHOOTING: {
                /*
                 * motion to directly hit target angle in the free zone
                 * by wrapping target angle to -180 to 180 degrees, following this motion
                 * CAN'T result in any sort of limit issue
                 * 
                 * can be trusted but will be unoptimized
                 */
                double freeMotion = robotTargetAngle - robotAngle;

                /*
                 * shortest possible movement to the target angle
                 * the result of this motion may be a faster path to the target angle,
                 * but also may pass over the limit, so it has to be checked
                 * 
                 * can't be trusted but will be optimized
                 */
                double a = (robotTargetAngle - robotAngle + Math.PI) % (2 * Math.PI);
                double b = (robotAngle - robotTargetAngle + Math.PI) % (2 * Math.PI);
                double shortestMotion = (a < b) ? -a : b;

                double motion = freeMotion;// shortestMotion;
                // fallback to free motion if shortest motion passes over limit
                if (robotAngle + shortestMotion > m_turretForwardSoftLimit || robotAngle + shortestMotion < m_turretReverseSoftLimit) {
                    motion = freeMotion;
                }

                return motion + robotAngle;
            }
            case FERRYING: {
                // i wrote two paragraphs explaining why this single statement is cool
                return robotTargetAngle;
            }
            case ROBOT_RELATIVE: {
                return targetAngle;
            }
        }

        return 0;
    }

    public void setTurretRobotRelative(boolean rr) {
        m_turretRobotRelative = rr;
    }

    public boolean getTurretRobotRelative() {
        return m_turretRobotRelative;
    }

    public void setTurretControl(boolean control) {
        m_turretControl = control;
    }

    public void enableTurretCalibration(TurretCalibration mode) {
        m_turretCalibrationEnabled = true;
        m_turretCalibratedForward = false;
        m_turretCalibration = mode;
    }

    // should only be used for debug purposes
    public void forceDisableTurretCalibration() {
        m_turretCalibrationEnabled = false;
    }

    public boolean isTurretCalibrating() {
        return m_turretCalibrationEnabled;
    }

    public void setTurretRawSpeed(double speed) {
        m_turretMotor.set(speed);
    }

    public void forceTurretZero() {
        m_turretMotor.getEncoder().setPosition(0);
        m_TDturretTargetAngle.set(0);
        m_TDturretSpeed.set(0);
        m_turretState = new TrapezoidProfile.State(0,0);
        m_turretSetpoint = new TrapezoidProfile.State(0,0);
    }

    private void runTurretCalibration() {
        switch (m_turretCalibration) {
        case CALIBRATE_FORWARD: {
            m_turretMotor.set(0.5);

            HardLimitDirection hardLimit = m_turretCurrentLimit.check();
            if (hardLimit == HardLimitDirection.kForward) {
                m_turretMotor.getEncoder().setPosition(m_turretForwardHardLimit);
                m_turretMotor.set(0);
                m_turretCalibrationEnabled = false;
            }

            break;
        }
        case CALIBRATE_FULL: {
            m_turretMotor.set(m_turretCalibratedForward ? -0.5 : 0.5);

            HardLimitDirection hardLimit = m_turretCurrentLimit.check();
            if (hardLimit == HardLimitDirection.kForward) {
                m_turretCalibratedForward = true;
                m_turretMotor.getEncoder().setPosition(0);
                m_turretMotor.set(-0.5);
            } else if (hardLimit == HardLimitDirection.kReverse && m_turretMotor.getEncoder().getPosition() < -Math.PI/2.0) {
                double range = m_turretMotor.getEncoder().getPosition();
                m_turretForwardHardLimit = -range/2.0;
                m_turretReverseHardLimit = range/2.0;
                m_turretForwardSoftLimit = m_turretForwardHardLimit - cfgDbl("turretSoftLimitWidth");
                m_turretReverseSoftLimit = m_turretReverseHardLimit + cfgDbl("turretSoftLimitWidth");

                m_turretMotor.getEncoder().setPosition(m_turretReverseHardLimit);
                m_turretState = new TrapezoidProfile.State(m_turretReverseHardLimit, 0);

                m_turretMotor.set(0);
                m_turretCalibrationEnabled = false;
            }
        }
        }
    }

    public Pose3d getTurretPose() {
        return m_turretPoseEstimator.getEstimatedPosition();
    }

    private void runTurret() {
        m_TDturretMeasuredPosition.set(m_turretMotor.getEncoder().getPosition());
        m_TDturretMeasuredVelocity.set(m_turretMotor.getEncoder().getVelocity());
        m_TDturretMeasuredCurrent.set(m_turretMotor.getOutputCurrent());
        m_TDturretProfilePosition.set(m_turretState.position);

        if (m_turretCalibrationEnabled) {
            runTurretCalibration();
            return;
        }

        if (!m_turretControl) return;

        if (m_tuneTurret) {
            if (m_TDturretP.get() != m_turretP ||
                    m_TDturretI.get() != m_turretI ||
                    m_TDturretD.get() != m_turretD) {
                m_turretP = m_TDturretP.get();
                m_turretI = m_TDturretI.get();
                m_turretD = m_TDturretD.get();

                m_turretConfig.closedLoop.pid(m_turretP, m_turretI, m_turretD);

                m_turretMotor.configure(m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }

            m_turretFF.setKs(m_TDturretKs.get());
            m_turretFF.setKv(m_TDturretKv.get());
            m_turretFF.setKa(m_TDturretKa.get());
        }

        m_TDturretTargetAngle.set(m_TDturretTargetAngle.get() + m_TDturretSpeed.get() * Constants.schedulerPeriodTime);

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        TurretState state = (m_tuneTurret || m_turretRobotRelative) ? TurretState.ROBOT_RELATIVE
                : (FieldUtils.getInstance().inAllianceZone(m_Drive.getPose(), alliance) ? TurretState.SHOOTING
                        : TurretState.FERRYING);
        double controlledAngle = angleToTarget(m_TDturretTargetAngle.get(), state);

        HardLimitDirection hardLimit = m_turretCurrentLimit.check();
        if (hardLimit == HardLimitDirection.kForward) {
            m_turretMotor.getEncoder().setPosition(m_turretForwardHardLimit);
            m_TDturretSpeed.set(0);
        } else if (hardLimit == HardLimitDirection.kReverse) {
            m_turretMotor.getEncoder().setPosition(m_turretReverseHardLimit);
            m_TDturretSpeed.set(0);
        }

        controlledAngle = MathUtil.clamp(controlledAngle, m_turretReverseSoftLimit, m_turretForwardSoftLimit);

        m_turretSetpoint = new TrapezoidProfile.State(controlledAngle, m_TDturretSpeed.get());

        double prevVelocity = m_turretState.velocity;
        m_turretState = m_turretProfile.calculate(Constants.schedulerPeriodTime, m_turretState, m_turretSetpoint);
        double turretFF = m_turretFF.calculateWithVelocities(prevVelocity, m_turretState.velocity);

        m_turretMotor.getClosedLoopController().setSetpoint(
                m_turretState.position, ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                turretFF);

        m_turretPoseEstimator.update(
            new Rotation3d(0, 0, 
                (Math.toRadians(m_Drive.getGyroAngle()))),
            new Rotation2d(m_turretMotor.getEncoder().getPosition()),
            m_Drive.getModulePositions());
        Optional<VisionEstimationResult> result = Vision.getInstance().getLatestFromCamera("TurretCamera");
        if (result.isPresent()) {
            m_turretPoseEstimator.addVisionMeasurement(result.get().estimatedPose, result.get().timestamp);

            VisionEstimationResult turretEstimation = result.get();

            if (!m_turretGotResult) m_turretPose = turretEstimation.estimatedPose;

            double slowT = Constants.schedulerPeriodTime * 2;
            double fastT = Constants.schedulerPeriodTime * 16;

            ChassisSpeeds delta = Drive.getInstance().getMeasuredSpeeds();
            Translation2d velocity = new Translation2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond);
            double vmag = velocity.getDistance(Translation2d.kZero);
            double vmax = Constants.DriveConstants.kMaxSpeedMetersPerSecond * 0.8 + Constants.DriveConstants.kMaxAngularSpeed * 0.25;
            double weight = MathUtil.clamp(vmag, 0, vmax)/vmax;

            double t = MathUtil.interpolate(slowT, fastT, weight);

            Translation3d smoothedPosition = m_turretPose.getTranslation().plus(
                turretEstimation.estimatedPose.getTranslation().minus(
                    m_turretPose.getTranslation()).times(t));
            Rotation3d smoothedRotation = m_turretPose.getRotation().plus(
                turretEstimation.estimatedPose.getRotation().minus(
                    m_turretPose.getRotation()).times(t));
            
            m_turretPose = new Pose3d(smoothedPosition, smoothedRotation);

            m_turretGotResult = true;
        }

        m_turretPoseField.setRobotPose(m_turretPoseEstimator.getEstimatedPosition().toPose2d());
        m_turretPoseField.getObject("Hub").setPose(FieldUtils.getInstance().getHubPose().toPose2d());
    }

    private void runHood() {
        if (m_tuneHood) {
            if (m_TDhoodP.get() != m_hoodP ||
                    m_TDhoodI.get() != m_hoodI ||
                    m_TDhoodD.get() != m_hoodD) {
                m_hoodP = m_TDhoodP.get();
                m_hoodI = m_TDhoodI.get();
                m_hoodD = m_TDhoodD.get();
                m_hoodConfig.closedLoop.pid(m_hoodP, m_hoodI, m_hoodD);
                m_hoodMotor.configure(m_hoodConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
            }
            m_hoodKg = m_TDhoodKg.get();
            m_hoodFF.setKg(m_hoodKg);
            m_hoodKs = m_TDhoodKs.get();
            m_hoodFF.setKs(m_hoodKs);
            m_hoodKv = m_TDhoodKv.get();
            m_hoodFF.setKv(m_hoodKv);

            m_hoodSetpoint = new TrapezoidProfile.State(m_TDhoodTargetPosition.get(), 0.0);
        }
        HardLimitDirection limit = m_hoodLimiter.check();
        double hoodAngle = m_hoodEncoder.getPosition();
        if (limit == HardLimitDirection.kForward) {
            if (m_hoodMotor.getAppliedOutput() > 0) {
                m_hoodMotor.set(0);
            }
            if (!MathUtil.isNear(Constants.ShooterConstants.kHoodMaxAngle, hoodAngle,
                    Constants.ShooterConstants.kHoodToleranceDegrees)) {
                m_hoodEncoder.setPosition(Constants.ShooterConstants.kHoodMaxAngle);
            }
            if (m_hoodSetpoint.position > hoodAngle) {
                m_hoodState = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMaxAngle, 0.0);
                m_hoodSetpoint = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMaxAngle, 0.0);
            }
        } else if (limit == HardLimitDirection.kReverse) {
            if (m_hoodMotor.getAppliedOutput() < 0) {
                m_hoodMotor.set(0);
            }
            if (!MathUtil.isNear(Constants.ShooterConstants.kHoodMinAngle, hoodAngle,
                    Constants.ShooterConstants.kHoodToleranceDegrees)) {
                m_hoodEncoder.setPosition(Constants.ShooterConstants.kHoodMinAngle);
            }
            if (m_hoodSetpoint.position < hoodAngle) {
                m_hoodState = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMinAngle, 0.0);
                m_hoodSetpoint = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMinAngle, 0.0);
            }
        }

        m_hoodState = m_hoodProfile.calculate(Constants.schedulerPeriodTime, m_hoodState, m_hoodSetpoint);
        double hoodFeedForward = m_hoodFF.calculate(m_hoodState.velocity);
        m_hoodClosedLoopController.setSetpoint(m_hoodState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                hoodFeedForward);

        m_TDHoodPosition.set(hoodAngle);
        m_TDHoodProfilePosition.set(m_hoodState.position);
        m_TDHoodMotorCurrent.set(m_hoodMotor.getOutputCurrent());
    }

    @Override
    public void periodic() {
        if (m_flywheelEnabled) {
            if (m_tuneFlywheel) {
                if (m_TDflywheelP.get() != m_flywheelP ||
                        m_TDflywheelI.get() != m_flywheelI ||
                        m_TDflywheelD.get() != m_flywheelD) {
                    m_flywheelP = m_TDflywheelP.get();
                    m_flywheelI = m_TDflywheelI.get();
                    m_flywheelD = m_TDflywheelD.get();

                    m_flywheelLeftConfig.closedLoop.pid(m_flywheelP, m_flywheelI, m_flywheelD);

                    m_flywheelLeftMotor.configure(m_flywheelLeftConfig, ResetMode.kResetSafeParameters,
                            PersistMode.kPersistParameters);
                }

                m_flywheelFF.setKv(m_TDflywheelKv.get());
                m_flywheelFF.setKa(m_TDflywheelKa.get());
                m_flywheelFF.setKs(m_TDflywheelKs.get());
            }

            double flywheelSetpoint = m_TDflywheelVelocity.get();
            if(flywheelSetpoint == 0) {
                m_flywheelLeftMotor.set(0);
            } else {
                double arbFF = m_flywheelFF.calculate(flywheelSetpoint);
                m_flywheelLeftMotor.getClosedLoopController().setSetpoint(flywheelSetpoint, ControlType.kVelocity,
                        ClosedLoopSlot.kSlot0, arbFF);
            }

            m_TDflywheelMeasuredVelocity.set(m_flywheelLeftMotor.getEncoder().getVelocity());
            m_TDflywheelMeasuredCurrent.set(m_flywheelLeftMotor.getOutputCurrent());
        }

        if (m_turretEnabled)
            runTurret();

        if (m_chimneyEnabled) {
            m_TDchimneyMeasuredCurrent.set(m_chimneyMotor.getAppliedOutput());
        }

        if (m_hoodEnabled) {
            runHood();
        }

        super.periodic();
    }
}
