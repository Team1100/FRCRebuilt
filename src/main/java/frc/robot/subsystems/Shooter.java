package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.sensing.SparkCurrentLimitDetector;
import frc.robot.utils.sensing.SparkCurrentLimitDetector.HardLimitDirection;

public class Shooter extends SubsystemBase{
    private static Shooter m_Shooter = null;

    // Flywheel
    private final boolean m_flywheelEnabled;
    private SparkFlex m_flywheelLeftMotor;
    private SparkFlex m_rightFlywheelMotor;

    private SparkFlexConfig m_flywheelLeftConfig;

    private double m_flywheelP;
    private double m_flywheelI;
    private double m_flywheelD;
    private TDNumber m_TDflywheelP;
    private TDNumber m_TDflywheelI;
    private TDNumber m_TDflywheelD;

    private TDNumber m_TDflywheelKS;
    private TDNumber m_TDflywheelKV;
    private TDNumber m_TDflywheelKA;

    private SimpleMotorFeedforward m_flywheelFF;

    private TDNumber m_TDflywheelVelocity;
    private TDNumber m_TDflywheelMeasuredVelocity;
    private TDNumber m_TDflywheelMeasuredCurrent;

    // Turret
    private final boolean m_turretEnabled;
  	private boolean m_tuneTurret;
    private SparkFlex m_turretMotor;

    private SparkFlexConfig m_turretConfig;

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
    private TDNumber m_TDturretMeasuredCurrent;

    //Hood
    private final boolean m_hoodEnabled;
    private boolean m_tuneHood;
    private SparkMax m_hoodAngleMotor;
    private SparkMaxConfig m_hoodConfig;

    private RelativeEncoder m_hoodEncoder;
    
    private double m_hoodKG;
    private double m_hoodKS;
    private double m_hoodKV;
    private double m_hoodP;
    private double m_hoodI;
    private double m_hoodD;
    private TDNumber m_TDhoodKG;
    private TDNumber m_TDhoodKS;
    private TDNumber m_TDhoodKV;
    private TDNumber m_TDhoodP;
    private TDNumber m_TDhoodI;
    private TDNumber m_TDhoodD;

    private SparkClosedLoopController m_hoodClosedLoopController;
    private ElevatorFeedforward m_hoodFF;
    private TrapezoidProfile m_hoodProfile;
    private TrapezoidProfile.State m_hoodState;
    private TrapezoidProfile.State m_hoodSetpoint;

    private SparkCurrentLimitDetector m_hoodLimiter;

    private TDNumber m_TDHoodMotorCurrent;
    private TDNumber m_TDHoodPosition;
    private TDNumber m_TDHoodProfilePosition;

    private Drive m_Drive;

	  // Chimney
    private final boolean m_chimneyEnabled;
	  private SparkFlex m_chimneyMotor;
	  private SparkFlexConfig m_chimneyConfig;

	  private TDNumber m_TDchimneyMeasuredCurrent;

    private Shooter(){
        super("Shooter");

        m_chimneyMotor = new SparkFlex(cfgInt("chimneyMotorCANid"), MotorType.kBrushless);

        m_flywheelEnabled = cfgBool("flywheelEnabled");
        m_turretEnabled = cfgBool("turretEnabled");
        m_hoodEnabled = cfgBool("hoodEnabled");
        m_chimneyEnabled = cfgBool("chimneyEnabled");

        if (m_flywheelEnabled) setupFlywheel();

        if (m_turretEnabled) setupTurret();
        
        if (m_hoodEnabled) setupHood();

		    if (m_chimneyEnabled) setupChimney();
    }

    public static Shooter getInstance() {
        if (m_Shooter == null) {
            m_Shooter = new Shooter();
        }
        return m_Shooter;
    } 

    public void setupFlywheel() {
        m_flywheelLeftMotor = new SparkFlex(cfgInt("leftFlywheelCANid"), MotorType.kBrushless);
        m_rightFlywheelMotor = new SparkFlex(cfgInt("rightFlywheelCANid"), MotorType.kBrushless);

        m_TDflywheelP = new TDNumber(this, "Flywheel", "P");
        m_TDflywheelI = new TDNumber(this, "Flywheel", "I");
        m_TDflywheelD = new TDNumber(this, "Flywheel", "D");
        m_TDflywheelP.set(cfgDbl("flywheelP"));
        m_TDflywheelI.set(cfgDbl("flywheelI"));
        m_TDflywheelD.set(cfgDbl("flywheelD"));
        m_flywheelP = m_TDflywheelP.get();
        m_flywheelI = m_TDflywheelI.get();
        m_flywheelD = m_TDflywheelD.get();

        m_TDflywheelKS = new TDNumber(this, "Flywheel", "Ks");
        m_TDflywheelKV = new TDNumber(this, "Flywheel", "Kv");
        m_TDflywheelKA = new TDNumber(this, "Flywheel", "Ka");
        m_TDflywheelKS.set(cfgDbl("flywheelKS"));
        m_TDflywheelKV.set(cfgDbl("flywheelKV"));
        m_TDflywheelKA.set(cfgDbl("flywheelKA"));

        m_flywheelLeftConfig = new SparkFlexConfig();
        m_flywheelLeftConfig.closedLoop.pid(m_flywheelP, m_flywheelI, m_flywheelD);
        m_flywheelLeftConfig.encoder.velocityConversionFactor(cfgDbl("flywheelVelocityFactor"));

        SparkFlexConfig rightConfig = new SparkFlexConfig();
        rightConfig.follow(cfgInt("leftFlywheelCANid"), true);
        m_rightFlywheelMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_flywheelFF = new SimpleMotorFeedforward(
            m_TDflywheelKV.get(), m_TDflywheelKS.get(), m_TDflywheelKA.get());

		    m_turretTolerance = cfgDbl("turretTolerance");

        m_TDflywheelVelocity = new TDNumber(this, "Flywheel", "Target Velocity");
        m_TDflywheelVelocity.set(0);

        m_TDflywheelMeasuredVelocity = new TDNumber(this, "Flywheel", "Measured Velocity");
        m_TDflywheelMeasuredCurrent = new TDNumber(this, "Flywheel", "Measured Current");
    }

    public void setupTurret() {
        m_tuneTurret = cfgBool("tuneTurret");
        m_turretMotor = new SparkFlex(cfgInt("turretRotationCANid"), MotorType.kBrushless);

        m_TDturretP = new TDNumber(this, "Turret", "P");
        m_TDturretI = new TDNumber(this, "Turret", "I");
        m_TDturretD = new TDNumber(this, "Turret", "D");

        m_TDturretP.set(cfgDbl("turretP"));
        m_TDturretI.set(cfgDbl("turretI"));
        m_TDturretD.set(cfgDbl("turretD"));
        m_turretP = m_TDturretP.get();
        m_turretI = m_TDturretI.get();
        m_turretD = m_TDturretD.get();

        m_turretConfig = new SparkFlexConfig();
        m_turretConfig.closedLoop.pid(m_turretP, m_turretI, m_turretD);
        m_turretConfig.idleMode(IdleMode.kBrake);
        m_turretConfig.encoder.positionConversionFactor(cfgDbl("turretPositionFactor"));
        m_turretConfig.closedLoop.positionWrappingEnabled(false);

        m_turretMotor.configure(m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Constraints constraints = new Constraints(cfgDbl("turretMaxVelocity"), cfgDbl("turretMaxAcceleration"));
        m_turretProfile = new TrapezoidProfile(constraints);

        m_TDturretKs = new TDNumber(this, "Turret", "Ks");
        m_TDturretKv = new TDNumber(this, "Turret", "Kv");
        m_TDturretKa = new TDNumber(this, "Turret", "Ka");

        m_TDturretKs.set(cfgDbl("turretKs"));
        m_TDturretKv.set(cfgDbl("turretKv"));
        m_TDturretKa.set(cfgDbl("turretKa"));

		    m_turretFF = new SimpleMotorFeedforward(m_TDturretKs.get(), m_TDturretKv.get(), m_TDturretKa.get());

        m_TDturretTargetAngle = new TDNumber(this, "Turret", "Target Angle");

        m_TDturretMeasuredPosition = new TDNumber(this, "Turret", "Measured Position");
        m_TDturretMeasuredCurrent = new TDNumber(this, "Turret", "Measured Current");

        m_Drive = Drive.getInstance();
    }

    public void setupHood() {
        m_tuneHood = cfgBool("tuneHood");
        m_hoodAngleMotor = new SparkMax(cfgInt("hoodAngleMotorCANid"), MotorType.kBrushless);

        m_TDhoodKG = new TDNumber(this, "Hood", "kG");
        m_TDhoodKS = new TDNumber(this, "Hood", "kS");
        m_TDhoodKV = new TDNumber(this, "Hood", "kV");
        m_TDhoodP = new TDNumber(this, "Hood", "P");
        m_TDhoodI = new TDNumber(this, "Hood", "I");
        m_TDhoodD = new TDNumber(this, "Hood", "D");

        m_TDhoodKG.set(cfgDbl("hoodkG"));
        m_TDhoodKS.set(cfgDbl("hoodkS"));
        m_TDhoodKV.set(cfgDbl("hoodkV"));
        m_TDhoodP.set(cfgDbl("hoodkP"));
        m_TDhoodI.set(cfgDbl("hoodkI"));
        m_TDhoodD.set(cfgDbl("hoodkD"));
        
        m_hoodKG = m_TDhoodKG.get();
        m_hoodKS = m_TDhoodKS.get();
        m_hoodKV = m_TDhoodKV.get();
        m_hoodP = m_TDhoodP.get();
        m_hoodI = m_TDhoodI.get();
        m_hoodD = m_TDhoodD.get();

        m_hoodConfig = new SparkMaxConfig();
        m_hoodConfig.idleMode(IdleMode.kBrake);
        m_hoodConfig.closedLoop
                .pid(m_hoodP, m_hoodI, m_hoodD)
                .positionWrappingEnabled(false);
        m_hoodConfig.encoder
            .positionConversionFactor(Constants.ShooterConstants.kHoodPositionConversion)
            .velocityConversionFactor(Constants.ShooterConstants.kHoodVelocityConversion);

        m_hoodAngleMotor.configure(m_hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_hoodEncoder = m_hoodAngleMotor.getEncoder();
        m_hoodClosedLoopController = m_hoodAngleMotor.getClosedLoopController();

        m_hoodLimiter = new SparkCurrentLimitDetector(
            m_hoodAngleMotor, 
            Constants.ShooterConstants.kHoodStallDetectCurrent, 
            Constants.ShooterConstants.kHoodStallSpeedTolerance);

        TrapezoidProfile.Constraints hoodConstraints = new TrapezoidProfile.Constraints(
            Constants.ShooterConstants.kHoodMaxVelocityDegreesPerSec, 
            Constants.ShooterConstants.kHoodMaxAcceleration);
        m_hoodProfile = new TrapezoidProfile(hoodConstraints);
        m_hoodState = new TrapezoidProfile.State(m_hoodEncoder.getPosition(), 0.0);
        m_hoodSetpoint = new TrapezoidProfile.State(m_hoodEncoder.getPosition(), 0.0);
        m_hoodFF = new ElevatorFeedforward(m_hoodKS, m_hoodKG, m_hoodKV);

        m_TDHoodMotorCurrent = new TDNumber(this, "Hood", "Motor Current");
        m_TDHoodPosition = new TDNumber(this, "Hood", "Position");
        m_TDHoodProfilePosition = new TDNumber(this, "Hood", "Profile Position");
    }

    public void setupChimney() {
        m_chimneyMotor = new SparkFlex(cfgInt("chimneyMotorCANid"), MotorType.kBrushless);
        m_chimneyConfig = new SparkFlexConfig();
        m_chimneyConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(cfgInt("chimneyRollerStallLimit"), cfgInt("chimneyRollerFreeLimit"));

        m_chimneyMotor.configure(m_chimneyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_TDchimneyMeasuredCurrent = new TDNumber(this, "Chimney", "Measured Current");
    }

    /**
     * Sets a target velocity for the flywheel.
     * @param rpm Velocity to approach in RPM.
     */
    public void setFlywheelTarget(double rpm) {
        m_TDflywheelVelocity.set(rpm);
    }

    /**
     * Sets a target position for the hood.
     * @param angle Position to approach in radians.
     */
    public void setHoodTarget(double angle) {
        double clampedAngle = MathUtil.clamp(angle, 
            Constants.ShooterConstants.kHoodMinAngle, 
            Constants.ShooterConstants.kHoodMaxAngle);
        m_hoodSetpoint = new TrapezoidProfile.State(clampedAngle, 0.0);
    }

    /**
     * Sets the target values of the turret.
     * @param targetAngle Target angle to approach in radians.
     * @param speed Speed to move target angle by in radians/second.
     */
    public void setTurretTarget(double targetAngle, double speed) {
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
        return MathUtil.isNear(m_TDturretTargetAngle.get(), m_turretMotor.getEncoder().getPosition(), m_turretTolerance);
    }

    public boolean flywheelAtTarget() {
        return MathUtil.isNear(m_TDflywheelVelocity.get(), m_turretMotor.getEncoder().getVelocity(), cfgDbl("flywheelTolerance"));
    }

    public boolean hoodAtTarget() {
        // TODO: implement hoodAtTarget
        return false;
    }

    /**
     * Gets a hood angle needed to throw the ball at a given pitch
     * @param angle Input pitch in radians (0 = +X, pi/2 = +Z)
     * @return Valid hood angle in radians
     */
    public double pitchToHood(double angle) {
        // TODO: implement pitch to hood angle conversion
        return 0;
    }

    /**
     * Convert a field velocity to a flywheel velocity for the flywheel
     * @param velocity Field velocity in m/s
     * @return Flywheel velocity in RPM
     */
    public double velocityToRPM(double velocity) {
        // TODO: implement velocity to RPM conversion
        return 0;
    }

    // see angleToTarget for more info
    enum TurretState {
        SHOOTING, // direct movement, take the shortest available movement
        FERRYING // indirect movement, try to get turret into the 'free range' closest to hub
    }
    /**
     * Get an angle relative to the field and transform it into an angle for
     * the turret to move to.
     * 
     * There's two different ways to optimize the turret's rotation:
     * first, move to the closest angle, and wrap at hard limits;
     * second, reduce interaction with the hard limits by staying in the 'free range' closest
     * to the hub.
     * 
     * <br></br>
     * 
     * <b>TurretState.SHOOTING</b> refers to the first optimization mode.
     * it simply calculates the shortest movement to the target angle, and if that passes
     * over the hard limit, then it instead wraps around over the hard limit, causing a
     * 360 degree rotation.
     * 
     * <br></br>
     * 
     * <b>TurretState.FERRYING</b> refers to the second optimization mode.
     * While the robot is ferrying, it'll follow a 'snaking' pattern, thus spending most
     * of the time with robot +X parallel to field Y-axis. This means there's a certain
     * 180 degree range which doesn't touch the hard limits, called the 'free range'.
     * When robot +X is parallel to field Y-axis, having the turret be in the hub-side
     * free range means it'll never have to cross over a limit, thus maintaining perfect
     * shortest-distance motions.
     * 
     * <br></br>
     * 
     * That's a lot of words for: always follow the version of the target angle that lies in the
     * range -180 to 180. When the turret is already in its hub-side free range, this will
     * result in the shortest distance. When the turret is on the corner, it'll invert
     * partway through the turn, but this turret should be quick enough to finish before
     * the corner chassis turning is even finished.
     * 
     * @param targetAngle Target angle in field orientation.
     * @param state Type of motion to optimize for.
     * @return Optimized target angle in robot orientation.
     */
    public double angleToTarget(double targetAngle, TurretState state) {
        // current target position, robot oriented
        double robotAngle = m_turretMotor.getEncoder().getPosition();

        Pose2d chassisPose = m_Drive.getPose();
        Rotation2d chassisRotation = chassisPose.getRotation();
        double chassisAngle = chassisRotation.getRadians();
        // current target angle, robot oriented
        double robotTargetAngle = targetAngle - chassisAngle;

        switch (state) {
            case SHOOTING: {
                double softLimit = cfgDbl("turretSoftLimit");

                /*
                 * motion to directly hit target angle in the free zone
                 * by wrapping target angle to -180 to 180 degrees, following this motion
                 * CAN'T result in any sort of limit issue
                 * 
                 * can be trusted but will be unoptimized
                 */
                double freeMotion = robotTargetAngle%Math.PI - robotAngle;

                /*
                 * shortest possible movement to the target angle
                 * the result of this motion may be a faster path to the target angle,
                 * but also may pass over the limit, so it has to be checked
                 * 
                 * can't be trusted but will be optimized
                 */
                double a = (robotTargetAngle - robotAngle) % Math.PI*2;
                double b = (robotAngle - robotTargetAngle) % Math.PI*2;
                double shortestMotion = (a < b) ? -a : b;

                double motion = shortestMotion;
                // fallback to free motion if shortest motion passes over limit
                if (robotAngle + shortestMotion > softLimit || robotAngle + shortestMotion < -softLimit) {
                    motion = freeMotion;
                }

                return motion + robotAngle;
            }
            case FERRYING: {
                // i wrote two paragraphs explaining why this single statement is cool
                return robotTargetAngle%Math.PI;
            }
        }

        return 0;
    }

    private void runTurret() {
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

        m_TDturretTargetAngle.set(m_TDturretTargetAngle.get() + m_TDturretSpeed.get() * 1/50.0);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        TurretState state = FieldUtils.getInstance().inAllianceZone(m_Drive.getPose(), alliance) ? TurretState.SHOOTING : TurretState.FERRYING;
        double controlledAngle = angleToTarget(m_TDturretTargetAngle.get(), state);
        m_turretSetpoint = new TrapezoidProfile.State(controlledAngle, m_TDturretSpeed.get());

        m_turretState = m_turretProfile.calculate(1/50.0, m_turretState, m_turretSetpoint);
        double turretFF = m_turretFF.calculate(m_turretState.velocity);

        m_turretMotor.getClosedLoopController().setSetpoint(
            controlledAngle, ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            turretFF);

        m_TDturretMeasuredPosition.set(m_turretMotor.getEncoder().getPosition());
        m_TDturretMeasuredCurrent.set(m_turretMotor.getAppliedOutput());
	  }

    private void runHood() {
        if(m_tuneHood) {
            if (m_TDhoodP.get() != m_hoodP ||
                m_TDhoodI.get() != m_hoodI ||
                m_TDhoodD.get() != m_hoodD) {
                    m_hoodP = m_TDhoodP.get();
                    m_hoodI = m_TDhoodI.get();
                    m_hoodD = m_TDhoodD.get();
                    m_hoodConfig.closedLoop.pid(m_hoodP, m_hoodI, m_hoodD);
                    m_hoodAngleMotor.configure(m_hoodConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }
            m_hoodKG = m_TDhoodKG.get();
            m_hoodFF.setKg(m_hoodKG);
            m_hoodKS = m_TDhoodKS.get();
            m_hoodFF.setKs(m_hoodKS);
            m_hoodKV = m_TDhoodKV.get();
            m_hoodFF.setKv(m_hoodKV);
        }
        HardLimitDirection limit = m_hoodLimiter.check();
        double hoodAngle = m_hoodEncoder.getPosition();
        if(limit == HardLimitDirection.kFree){
            m_hoodState = m_hoodProfile.calculate(Constants.schedulerPeriodTime, m_hoodState, m_hoodSetpoint);
            double hoodFeedForward = m_hoodFF.calculate(m_hoodState.velocity);
            m_hoodClosedLoopController.setSetpoint(m_hoodState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, hoodFeedForward);
        }
        else if(limit == HardLimitDirection.kForward) {
            if(m_hoodAngleMotor.getAppliedOutput() > 0){
                m_hoodAngleMotor.set(0);
            }
            if(!MathUtil.isNear(Constants.ShooterConstants.kHoodMaxAngle, hoodAngle, Constants.ShooterConstants.kHoodToleranceDegrees)){
                m_hoodEncoder.setPosition(Constants.ShooterConstants.kHoodMaxAngle);
            }
            if(m_hoodSetpoint.position > hoodAngle)
            {
                m_hoodState = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMaxAngle, 0.0);
                m_hoodSetpoint = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMaxAngle, 0.0);
            }
        } 
        else if (limit == HardLimitDirection.kReverse) {
            if(m_hoodAngleMotor.getAppliedOutput() < 0){
                m_hoodAngleMotor.set(0);
            }
            if(!MathUtil.isNear(Constants.ShooterConstants.kHoodMinAngle, hoodAngle, Constants.ShooterConstants.kHoodToleranceDegrees)){
                m_hoodEncoder.setPosition(Constants.ShooterConstants.kHoodMinAngle);
            }
            if(m_hoodSetpoint.position < hoodAngle)
            {
                m_hoodState = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMinAngle, 0.0);
                m_hoodSetpoint = new TrapezoidProfile.State(Constants.ShooterConstants.kHoodMinAngle, 0.0);
            }
        }

        m_TDHoodPosition.set(hoodAngle);
        m_TDHoodProfilePosition.set(m_hoodState.position);
        m_TDHoodMotorCurrent.set(m_hoodAngleMotor.getOutputCurrent());
    }
    
    @Override
    public void periodic() {
        if (cfgBool("tuneFlywheelPID")) {
            if (m_TDflywheelP.get() != m_flywheelP ||
                m_TDflywheelI.get() != m_flywheelI ||
                m_TDflywheelD.get() != m_flywheelD) {
                m_flywheelP = m_TDflywheelP.get();
                m_flywheelI = m_TDflywheelI.get();
                m_flywheelD = m_TDflywheelD.get();

                m_flywheelLeftConfig.closedLoop.pid(m_flywheelP, m_flywheelI, m_flywheelD);

                m_flywheelLeftMotor.configure(m_flywheelLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }
        }

        if (m_flywheelEnabled) {
            double arbFF = m_flywheelFF.calculate(m_TDflywheelVelocity.get());
            m_flywheelLeftMotor.getClosedLoopController().setSetpoint(m_TDflywheelVelocity.get(), ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFF);

            m_TDflywheelMeasuredVelocity.set(m_flywheelLeftMotor.getEncoder().getVelocity());
            m_TDflywheelMeasuredCurrent.set(m_flywheelLeftMotor.getOutputCurrent());
        }

        if (m_turretEnabled) runTurret();

        if (m_chimneyEnabled) {
          m_TDchimneyMeasuredCurrent.set(m_chimneyMotor.getAppliedOutput());
        }

        if(m_hoodEnabled) {
            runHood();
        }

        super.periodic();
    }
}
