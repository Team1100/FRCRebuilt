package frc.robot.commands.shooter;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.FieldUtils;

public class FerryShoot extends Command {
    private final Shooter m_Shooter;
    private final FieldUtils m_FieldUtils;
    private final OI m_OI;
    
    private final ShootToPose m_internalShootToPose;

    private Translation2d m_targetPosition;

    private static Field2d m_field;

    public FerryShoot() {
        super(Shooter.getInstance(), "Targeted Shooting", "FerryShoot");
        m_Shooter = Shooter.getInstance();
        m_FieldUtils = FieldUtils.getInstance();
        m_OI = OI.getInstance();
        addRequirements(m_Shooter);

        if (m_field == null) {
            m_field = new Field2d();
            m_field.setRobotPose(new Pose2d(new Translation2d(-10, 0), Rotation2d.kZero));
            new TDSendable(m_Shooter, "Targeted Shooting", "FerryTargetDisplay", m_field);
        }

        m_internalShootToPose = ShootToPose.withFixedAngle(this::getTargetPosition, Math.toRadians(40));
    }

    public Pose3d getTargetPosition() {
        return new Pose3d(new Translation3d(m_targetPosition), Rotation3d.kZero);
    }

    @Override
    public void initialize() {
        Pose2d chassisPose = Drive.getInstance().getPose();
        Translation2d target;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double midFieldY = FieldUtils.getInstance().getHubPose().getTranslation().getY();
        if ((alliance == Alliance.Blue && chassisPose.getY() < midFieldY) || 
            (alliance == Alliance.Red && chassisPose.getY() > midFieldY)) {
            Pose3d trenchPose = FieldUtils.getInstance().getTagPose(FieldUtils.getInstance().getAllianceAprilTags().frontRightTrench);
            Pose3d hitPose = trenchPose.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(3.3, -1, 0), Rotation3d.kZero)));
            target = hitPose.getTranslation().toTranslation2d();
        } else {
            Pose3d trenchPose = FieldUtils.getInstance().getTagPose(FieldUtils.getInstance().getAllianceAprilTags().frontLeftTrench);
            Pose3d hitPose = trenchPose.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(3.3, 1, 0), Rotation3d.kZero)));
            target = hitPose.getTranslation().toTranslation2d();
        }
        m_targetPosition = target;
        m_internalShootToPose.initialize();
    }

    @Override
    public void execute() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double x = m_OI.getOperatorController().getRightY();
        double y = -m_OI.getOperatorController().getRightX();
        Translation2d offset = new Translation2d(MathUtil.applyDeadband(x, 0.05), MathUtil.applyDeadband(y, 0.05));
        if (alliance == Alliance.Red) offset = offset.unaryMinus();

        m_targetPosition = m_targetPosition.plus(offset.times(Constants.schedulerPeriodTime));
        m_field.getRobotObject().setPose(new Pose2d(m_targetPosition, Rotation2d.kZero));
        m_internalShootToPose.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_internalShootToPose.end(interrupted);
    }
}
