package frc.robot;


import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Configuration;
import frc.robot.utils.vision.VisionConfig;

public final class Constants {
    public static final String robotName = "robot360";
    public static final double schedulerPeriodTime = 0.02;

    public static final class SwerveModuleConstants {
        public static final boolean kTurningEncoderInverted = true;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kTurningMotorCurrentLimit = 20;
        public static final int kDrivingMotorCurrentLimit = 80;

        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final int kDrivingMotorPinionTeeth = 13;
        public static final double kDrivingMotorReduction = (45.0*22)/(kDrivingMotorPinionTeeth*15);

        public static final double kVortexFreeSpeedRPM = 6784;
        public static final double kDrivingMotorFreeSpeedRPS = kVortexFreeSpeedRPM / 60;
        public static final double kDriveWheelFreeSpeedRPS = (kDrivingMotorFreeSpeedRPS * kWheelCircumferenceMeters) / kDrivingMotorReduction;
        static double nominalVoltage = 12.0;
        public static final double kDrivingVelocityFF = 1.0 / kDriveWheelFreeSpeedRPS;


        public static final double kDrivingP = Configuration.getInstance().getDouble("Drive", "kDrivingP");
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRPS;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = Configuration.getInstance().getDouble("Drive", "kTurningP");
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;

        public static final double kTurningEncoderPositionFactor = 2 * Math.PI;
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
        public static final double kTurningEncoderPositionPIDMinInput = 0;
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;
    }

    public static final class DriveConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.92;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        // TODO: correct module positions
        public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
        public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
        public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
        public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

        public static final Translation2d[] kModuleLocations = new Translation2d[] {
            kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation
        };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation
        );

        public static final double kFrontLeftChassisAngularOffset = -Math.PI/2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI/2;

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        public static final double kBaseRadius = Units.inchesToMeters(RobotMap.R_BASE_RADIUS_INCHES);
        public static final boolean kGyroReversed = false;
    }
    public static final class IntakeConstants{
        public static final int kIntakeID = -1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.92;
        public static final double kMaxAccelerationMetersPerSecondSquared = 10;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;


        public static final double kPathFollowerMaxSpeed = AutoConstants.kMaxSpeedMetersPerSecond; // Max module speed, in m/s
        public static final double kPathFollowerBaseRadius = DriveConstants.kBaseRadius; // Drive base radius in meters
        public static final double kPathFollowerMass = 52.1631; // 115 pounds
        public static final double kPathFollowerMomentOfInertia = 6.2; // Total guess. Rough estimate of l^2 + w^2 * m * 1/12
        public static final double kPathFollowerWheelCoeficientFriction = 1.2; // Total guess. pathplaner default

        public static final PIDConstants kPathFollowerTranslationPID = new PIDConstants(5.0, 0.0, 0.0); // Translation PID constants
        public static final PIDConstants kPathFollowerRotationPID = new PIDConstants(5.0, 0.0, 0.0); // Rotation PID constants    
    }

    public static final class FieldLocationConstants {
        // calculated from values in glass
        public static final double kMidfieldX = Units.feetToMeters(27.13);
        public static final double kRedAllianceZoneX = Units.feetToMeters(39.0);
        public static final double kBlueAllianceZoneX = Units.feetToMeters(15.26);
    }

    public static final class VisionConstants {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // TODO: experiment to find actual stddevs
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
        // Maximum ambiguity accepted as a valid result from the vision systems
        public static final double kMaxValidAmbiguity = 0.2;
        public static final double kMaxZError = 0.75;
        public static final double kMaxRollError = 0.5;
        public static final double kMaxPitchError = 0.5;

        public static final VisionConfig[] kLeafletVisionSystems = null;
        public static final VisionConfig[] kRebuiltVisionSystems = null;
    }

    public static class ShooterConstants {
        public static final double kVelocityFactor = 1;

        public static final double kHoodMotorFreeSpeed = Configuration.getInstance().getDouble("Shooter", "hoodMotorFreeSpeed");
        public static final double kHoodPlanetaryRatio = Configuration.getInstance().getDouble("Shooter", "hoodPlanetaryRatio");
        public static final double kHoodPinionTeeth = Configuration.getInstance().getDouble("Shooter", "hoodPinionTeeth");
        public static final double kHoodLanternTeeth = Configuration.getInstance().getDouble("Shooter", "hoodLanternTeeth");

        public static final double kHoodPositionConversion = 360 * kHoodPinionTeeth / (kHoodPlanetaryRatio * kHoodLanternTeeth);
        public static final double kHoodVelocityConversion = kHoodPositionConversion / 60;
        public static final double kHoodMaxVelocityDegreesPerSec = kHoodMotorFreeSpeed * kHoodVelocityConversion;
        public static final double kHoodMaxAcceleration = Configuration.getInstance().getDouble("Shooter", "hoodMaxAcceleration");
        public static final double kHoodStallDetectCurrent = 5; //amps
        public static final double kHoodStallSpeedTolerance = 5; //Degrees per second
        public static final double kHoodMaxAngle = Configuration.getInstance().getDouble("Shooter", "hoodMaxAngle");
        public static final double kHoodMinAngle = Configuration.getInstance().getDouble("Shooter", "hoodMinAngle");
        public static final double kHoodToleranceDegrees = 1.0;
    }

    public static final class OIConstants {
        public static final double kDriveDeadband = 0.05;
    }

    public static final class ClimberConstants {
        // Climber Constants
        public static final boolean kEnableClimberPIDTuning = false;
        public static final boolean kEnableClimberClosedLoopControl = true;
        public static final double kClimberP = 0.3;
        public static final double kClimberI = 0;
        public static final double kClimberD = 0;
        public static final double kClimberkS = 0.20;
        public static final double kClimberkG = 0.07;
        public static final double kClimberkV = 0.435;
        public static final double kClimberkA = 0.0;
        public static final double kClimberToleranceInches = 0.3;

        public static final double kClimberMaxVelocity = 35;
        public static final double kClimberMaxAcceleration = 35;
        public static final double kClimberSpeed = 0.2;
        public static final double kClimberSpeedRPM = 30;

        public static final double kClimberEncoderPositionFactor = (20.0/22.0*0.25); /* (planetary ratio) / (sprocket teeth) * (Inches per tooth) */

        public static final double DEGREES_PER_REVOLUTION = 360;
        public static final double kClimberLowerLimitInches = 0;
        public static final double kClimberUpperLimitInches = 26.5;
    }

    public static final class LEDConstants {
        public static final int LEDLength = 49; // number of LEDs
        public static final int kPositionSplitIndex = 10;
        public static final boolean kInversePolarity = true; //dont know what it does but it seems cool
    }
}
