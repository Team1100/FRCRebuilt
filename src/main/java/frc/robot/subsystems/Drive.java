// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.math.Matrix;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.testingdashboard.*;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.FieldUtils;

public class Drive extends SubsystemBase {
  private static Drive m_Drive;
  private final Field2d m_field;

  private final MAXSwerveModule m_FrontLeft = new MAXSwerveModule(
    config().getMotorController("frontLeftDrive"),
    config().getMotorController("frontLeftTurning"),
    Constants.DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_FrontRight = new MAXSwerveModule(
    config().getMotorController("frontRightDrive"),
    config().getMotorController("frontRightTurning"),
    Constants.DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_BackLeft = new MAXSwerveModule(
    config().getMotorController("backLeftDrive"),
    config().getMotorController("backLeftTurning"),
    Constants.DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_BackRight = new MAXSwerveModule(
    config().getMotorController("backRightDrive"),
    config().getMotorController("backRightTurning"),
    Constants.DriveConstants.kBackRightChassisAngularOffset);
  
  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  //Slew rate filter variable
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private ChassisSpeeds m_lastSpeeds = new ChassisSpeeds();

  private final Config cfg = new Config();
  private final Consumer<Voltage> out = (volts)->{drive(volts.magnitude(), 0, 0, false, false);};
  private final Consumer<SysIdRoutineLog> log = entry->{
    entry.motor("FRD").voltage(m_FrontRight.getDriveVoltage())
                      .linearPosition(m_FrontRight.getDrivePosition())
                      .linearVelocity(m_FrontRight.getDriveVelocity());
    entry.motor("FLD").voltage(m_FrontLeft.getDriveVoltage())
                      .linearPosition(m_FrontLeft.getDrivePosition())
                      .linearVelocity(m_FrontLeft.getDriveVelocity());
    entry.motor("BRD").voltage(m_BackRight.getDriveVoltage())
                      .linearPosition(m_BackRight.getDrivePosition())
                      .linearVelocity(m_BackRight.getDriveVelocity());
    entry.motor("BLD").voltage(m_BackLeft.getDriveVoltage())
                      .linearPosition(m_BackLeft.getDrivePosition())
                      .linearVelocity(m_BackLeft.getDriveVelocity());
  };
  private Mechanism mech = new Mechanism(out, log, this, "Swerve");
  private SysIdRoutine m_sysid = new SysIdRoutine(cfg, mech);

  //SwerveCrivePoseEstimator class for tracking robot pose
  SwerveDrivePoseEstimator m_DrivePoseEstimator;

  //testing dashboard instances
  TDNumber TDxSpeedCommanded;
  TDNumber TDySpeedCommanded;
  TDNumber TDrotSpeedCommanded;

  TDNumber TDFrontLeftDriveSpeed;
  TDNumber TDFrontLeftAngle;
  TDNumber TDFrontRightDriveSpeed;
  TDNumber TDFrontRightAngle;
  TDNumber TDBackLeftDriveSpeed;
  TDNumber TDBackLeftAngle;
  TDNumber TDBackRightDriveSpeed;
  TDNumber TDBackRightAngle;

  TDNumber TDxSpeedMeasured;
  TDNumber TDySpeedMeasured;
  TDNumber TDrotSpeedMeasured;

  TDNumber TDPoseX;
  TDNumber TDPoseY;
  TDNumber TDPoseAngle;
  TDNumber TDGyroAngle;

  ChassisSpeeds m_requestSpeeds = new ChassisSpeeds();
  ChassisSpeeds m_limitSpeeds = new ChassisSpeeds();
  double m_driveTime = 0;

  TDNumber m_DLeftFrontCurrentOutput;
  TDNumber m_DRightFrontCurrentOutput;
  TDNumber m_DLeftBackCurrentOutput;
  TDNumber m_DRightBackCurrentOutput;

  TDNumber m_TLeftFrontCurrentOutput;
  TDNumber m_TRightFrontCurrentOutput;
  TDNumber m_TLeftBackCurrentOutput;
  TDNumber m_TRightBackCurrentOutput;

  TDNumber m_TRightFrontSetpoint;
  TDNumber m_TLeftFrontSetpoint;
  TDNumber m_TRightBackSetpoint;
  TDNumber m_TLeftBackSetpoint;

  /** Creates a new Drive. */
  private Drive() {
    super("Drive");
    m_DrivePoseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kinematics, 
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), 
      new SwerveModulePosition[] {
        m_FrontLeft.getPosition(),
        m_FrontRight.getPosition(),
        m_BackLeft.getPosition(),
        m_BackRight.getPosition()
      }, new Pose2d());

    m_field = new Field2d();
    new TDSendable(this, "Field", "Position", m_field);

    TDxSpeedCommanded = new TDNumber(this, "Drive Input", "XInputSpeed");
    TDySpeedCommanded = new TDNumber(this, "Drive Input", "YInputSpeed");
    TDrotSpeedCommanded = new TDNumber(this, "Drive Input", "RotInputSpeed");

    TDxSpeedMeasured = new TDNumber(this, "Drive Speed", "XMeasuredSpeed");
    TDySpeedMeasured = new TDNumber(this, "Drive Speed", "YMeasuredSpeed");
    TDrotSpeedMeasured = new TDNumber(this, "Drive Speed", "RotMeasuredSpeed");

    TDFrontLeftDriveSpeed = new TDNumber(this, "Drive Speed", "Front Left Speed");
    TDFrontLeftAngle = new TDNumber(this, "Drive Speed", "Front Left Angle");
    TDFrontRightDriveSpeed = new TDNumber(this, "Drive Speed", "Front Right Speed");
    TDFrontRightAngle = new TDNumber(this, "Drive Speed", "Front Right Angle");
    TDBackLeftDriveSpeed = new TDNumber(this, "Drive Speed", "Back Left Speed");
    TDBackLeftAngle = new TDNumber(this, "Drive Speed", "Back Left Angle");
    TDBackRightDriveSpeed = new TDNumber(this, "Drive Speed", "Back Right Speed");
    TDBackRightAngle = new TDNumber(this, "Drive Speed", "Back Right Angle");
  
    
    TDPoseX = new TDNumber(this, "Drive Pose", "PoseX");
    TDPoseY = new TDNumber(this, "Drive Pose", "PoseY");
    TDPoseAngle = new TDNumber(this, "Drive Pose", "PoseAngle");
    TDGyroAngle = new TDNumber(this, "Drive Pose", "GyroAngle");

    m_DLeftFrontCurrentOutput = new TDNumber(this, "Current", "Drive Front Left Output");
    m_DRightFrontCurrentOutput = new TDNumber(this, "Current", "Drive Front Right Output");
    m_DLeftBackCurrentOutput = new TDNumber(this, "Current", "Drive Back Left Output");
    m_DRightBackCurrentOutput = new TDNumber(this, "Current", "Drive Back Right Output");
    
    m_TLeftFrontCurrentOutput = new TDNumber(this, "Current", "Steering Front Left Output");
    m_TRightFrontCurrentOutput = new TDNumber(this, "Current", "Steering Front Right Output");
    m_TLeftBackCurrentOutput = new TDNumber(this, "Current", "Steering Back Left Output");
    m_TRightBackCurrentOutput = new TDNumber(this, "Current", "Steering Back Right Output");  
  
    m_TRightFrontSetpoint = new TDNumber(this, "Current", "Speed Front Right Setpoint");
    m_TLeftFrontSetpoint  = new TDNumber(this, "Current", "Speed Front Left Setpoint");
    m_TRightBackSetpoint  = new TDNumber(this, "Current", "Speed Back Right Setpoint");
    m_TLeftBackSetpoint   = new TDNumber(this, "Current", "Speed Back Left Setpoint");
    DCMotor neovortex = DCMotor.getNeoVortex(1).withReduction(SwerveModuleConstants.kDrivingMotorReduction);
    
    ModuleConfig swerveModuleConfig = new ModuleConfig(SwerveModuleConstants.kWheelDiameterMeters/2, DriveConstants.kMaxSpeedMetersPerSecond, 
    AutoConstants.kPathFollowerWheelCoeficientFriction, neovortex, SwerveModuleConstants.kDrivingMotorCurrentLimit, 4);
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getMeasuredSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new PPHolonomicDriveController(AutoConstants.kPathFollowerTranslationPID, AutoConstants.kPathFollowerRotationPID),
      new RobotConfig(AutoConstants.kPathFollowerMass, AutoConstants.kPathFollowerMomentOfInertia, swerveModuleConfig, DriveConstants.kModuleLocations),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
    PathPlannerLogging.setLogCurrentPoseCallback((pose)->{
      m_field.getObject("AutoCurrent").setPose(pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose)->{
      m_field.getObject("AutoTarget").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses)->{
      m_field.getObject("Auto Active Path").setPoses(poses);
    });
  }

  public static Drive getInstance() {
    if(m_Drive == null){
      m_Drive = new Drive();
    }

    return m_Drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_DrivePoseEstimator.update(
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), 
      new SwerveModulePosition[] {
        m_FrontLeft.getPosition(),
        m_FrontRight.getPosition(),
        m_BackLeft.getPosition(),
        m_BackRight.getPosition()
      });
    updateTD();
    super.periodic();
  }

  public void simulationPeriodic() {
    double now = Timer.getFPGATimestamp();

    Pose2d lastPose = getPose();
    if (m_driveTime != 0 && m_limitSpeeds != null) {
      ChassisSpeeds fieldRelativSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_limitSpeeds, lastPose.getRotation());
      double deltax = fieldRelativSpeeds.vxMetersPerSecond * (now - m_driveTime);
      double deltay = fieldRelativSpeeds.vyMetersPerSecond * (now - m_driveTime);
      Rotation2d rot = new Rotation2d(fieldRelativSpeeds.omegaRadiansPerSecond * (now - m_driveTime));
      Translation2d translation = new Translation2d(deltax, deltay);
      Pose2d newPose = new Pose2d(
        lastPose.getTranslation().plus(translation), 
        lastPose.getRotation().plus(rot));
      resetOdometry(newPose);
                
    }

    m_driveTime = now;

  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return Current robot pose.
   */
  public Pose2d getPose() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_FrontLeft.getPosition(),
      m_FrontRight.getPosition(),
      m_BackLeft.getPosition(),
      m_BackRight.getPosition()
    };
  }

  public ChassisSpeeds getMeasuredSpeeds(){
    if(RobotBase.isReal()) {
      SwerveModuleState[] moduleStates = new SwerveModuleState[4];
      moduleStates[0] = m_FrontLeft.getState();
      moduleStates[1] = m_FrontRight.getState();
      moduleStates[2] = m_BackLeft.getState();
      moduleStates[3] = m_BackRight.getState();

      return DriveConstants.kinematics.toChassisSpeeds(moduleStates);
    }
    else{
      return m_limitSpeeds;
    }
  }

  /**
   * Updates pose estimator with estimate from the Vision subsystem.
   * 
   * @param pose Current pose of the robot.
   * @param timestamp When the vision measurement was taken.
   * @param stdDevs Confidence.
   */
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    m_DrivePoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  /*
   * Resets the odometry to the specified pose
   * 
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    m_DrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), 
    new SwerveModulePosition[] {
      m_FrontLeft.getPosition(),
      m_FrontRight.getPosition(),
      m_BackLeft.getPosition(),
      m_BackRight.getPosition()
    }, 
    pose);
  }

  /*
   * Method to drive the robot using joystick info.
   * 
   * @param xSpeed - Speed of the robot in the x direction (forward)
   * @param ySpeed - Speed of the robot in the y direction (sideways)
   * @param rot    - Angular rate of the robot
   * @param fieldRelative - Whether the provided x and y speeds are relative to the field
   * @param rateLimit - whether to enable rate limiting for smoother control
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    
    if(rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      //Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if(m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      }
      else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowards(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > .85*Math.PI) {
        if(m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);
    }
    else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    //Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    Rotation2d fieldAngle = m_DrivePoseEstimator.getEstimatedPosition().getRotation()
                            .plus(FieldUtils.getInstance().getRotationOffset()); //Compensates for alliance
    
    drive(
      fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldAngle)
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );
  }

  /*
   * Sets the wheel into an X formation to prevent movement
   */
  public void setX() {
    m_FrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_FrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_BackLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_BackRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  /*
   * Sets the swerve ModuleStates
   * 
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_FrontLeft.setDesiredState(desiredStates[0]);
    m_FrontRight.setDesiredState(desiredStates[1]);
    m_BackLeft.setDesiredState(desiredStates[2]);
    m_BackRight.setDesiredState(desiredStates[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    TDxSpeedCommanded.set(speeds.vxMetersPerSecond);
    TDySpeedCommanded.set(speeds.vyMetersPerSecond);
    TDrotSpeedCommanded.set(speeds.omegaRadiansPerSecond);
    m_requestSpeeds = speeds;
    ChassisSpeeds limitSpeeds = limitRates(speeds);
    m_limitSpeeds = limitSpeeds;
    var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(limitSpeeds);
    setModuleStates(swerveModuleStates);
  }

  //resets the drive encoders to currently read a position pf 0.
  public void resetEncoders() {
    m_FrontLeft.resetEncoders();
    m_FrontRight.resetEncoders();
    m_BackLeft.resetEncoders();
    m_BackRight.resetEncoders();
  }

  //Zeros the heading of the robot
  public void zeroHeading() {
    m_gyro.reset();
  }

  /*
   * Returns the heading of the robot.
   * 
   * @return the robot's heading in degrees
   */
  public double getHeading() {
    return getPose().getRotation().getDegrees();
  }

  public double getGyroAngle() {
    return m_gyro.getAngle(IMUAxis.kZ);
  }

  /*
   * Returns the turn rate of the robot.
   * 
   * @return The turn rate of the robot, in degree per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysid.quasistatic(direction);
  }
  public edu.wpi.first.wpilibj2.command.Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysid.dynamic(direction);
  }

  private void updateTD(){

    m_field.setRobotPose(getPose());

    ChassisSpeeds measuredSpeeds =  getMeasuredSpeeds();
    TDxSpeedMeasured.set(measuredSpeeds.vxMetersPerSecond);
    TDySpeedMeasured.set(measuredSpeeds.vyMetersPerSecond);
    TDrotSpeedMeasured.set(measuredSpeeds.omegaRadiansPerSecond);

    Pose2d currentPose = getPose();
    TDPoseX.set(currentPose.getX());
    TDPoseY.set(currentPose.getY());
    TDPoseAngle.set(currentPose.getRotation().getDegrees());
    TDGyroAngle.set(getGyroAngle());

    SwerveModuleState fl = m_FrontLeft.getState();
    TDFrontLeftDriveSpeed.set(fl.speedMetersPerSecond);
    TDFrontLeftAngle.set(fl.angle.getDegrees());
    SwerveModuleState fr = m_FrontRight.getState();
    TDFrontRightDriveSpeed.set(fr.speedMetersPerSecond);
    TDFrontRightAngle.set(fr.angle.getDegrees());
    SwerveModuleState bl = m_BackLeft.getState();
    TDBackLeftDriveSpeed.set(bl.speedMetersPerSecond);
    TDBackLeftAngle.set(bl.angle.getDegrees());
    SwerveModuleState br = m_BackRight.getState();
    TDBackRightDriveSpeed.set(br.speedMetersPerSecond);
    TDBackRightAngle.set(br.angle.getDegrees());

    m_DLeftFrontCurrentOutput.set(m_FrontLeft.getDriveOutputCurrent());
    m_DRightFrontCurrentOutput.set(m_FrontRight.getDriveOutputCurrent());
    m_DLeftBackCurrentOutput.set(m_BackLeft.getDriveOutputCurrent());
    m_DRightBackCurrentOutput.set(m_BackRight.getDriveOutputCurrent());

    m_TLeftFrontCurrentOutput.set(m_FrontLeft.getTurningOutputCurrent());
    m_TRightFrontCurrentOutput.set(m_FrontRight.getTurningOutputCurrent());
    m_TLeftBackCurrentOutput.set(m_BackLeft.getTurningOutputCurrent());
    m_TRightBackCurrentOutput.set(m_BackRight.getTurningOutputCurrent());

    m_TRightFrontSetpoint.set(m_FrontRight.getLastSpeed());
    m_TLeftFrontSetpoint.set(m_FrontLeft.getLastSpeed());
    m_TRightBackSetpoint.set(m_BackRight.getLastSpeed());
    m_TLeftBackSetpoint.set(m_BackLeft.getLastSpeed());
  }

  public ChassisSpeeds limitRates(ChassisSpeeds commandedSpeeds) {
    //Apply Speed Limits
    double linearSpeed = Math.hypot(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
    Rotation2d direction = Rotation2d.kZero;
    if(Math.abs(commandedSpeeds.vxMetersPerSecond) > 1e-6 ||
       Math.abs(commandedSpeeds.vyMetersPerSecond) > 1e-6){
      direction = new Rotation2d(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
    }
    double limitedSpeed = Math.min(linearSpeed, DriveConstants.kMaxSpeedMetersPerSecond);
    double limitedTheta = MathUtil.clamp(commandedSpeeds.omegaRadiansPerSecond, 
                                            -DriveConstants.kMaxAngularSpeed, 
                                            DriveConstants.kMaxAngularSpeed);
    
    Translation2d linearVelocity = (new Pose2d(new Translation2d(), direction)
                                      .transformBy(new Transform2d(new Translation2d(limitedSpeed, 0.0), new Rotation2d()))).getTranslation();
                        
    //Apply Acceleration Limits
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double accelerationDif = AutoConstants.kMaxAccelerationMetersPerSecondSquared * (currentTime-m_prevTime);
    double xSpeed = MathUtil.clamp(linearVelocity.getX(),
                                  m_lastSpeeds.vxMetersPerSecond - accelerationDif,
                                  m_lastSpeeds.vxMetersPerSecond + accelerationDif);

    double ySpeed = MathUtil.clamp(linearVelocity.getY(),
                                 m_lastSpeeds.vyMetersPerSecond - accelerationDif,
                                 m_lastSpeeds.vyMetersPerSecond + accelerationDif);

    double thetaAccelDif = AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared * (currentTime - m_prevTime);
    double thetaSpeed = MathUtil.clamp(limitedTheta,
                                 m_lastSpeeds.omegaRadiansPerSecond - thetaAccelDif,
                                 m_lastSpeeds.omegaRadiansPerSecond + thetaAccelDif);

   ChassisSpeeds limitedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
   m_lastSpeeds = limitedSpeeds;
   m_prevTime = currentTime;
   return limitedSpeeds;
  }
}