package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.vision.VisionConfig;
import frc.robot.utils.vision.VisionEstimationResult;

public class TurretToHub extends Command{
    private final Shooter m_Shooter;
    private final Vision m_Vision;

    private Transform3d m_cameraToTurret;
    private Transform3d m_chassisToTurret;

    private Pose3d m_hubPose;

    private String m_cameraName;

    public TurretToHub() {
        super(Shooter.getInstance(), "Turret", "TurretToHub");

        m_Shooter = Shooter.getInstance();
        m_Vision = Vision.getInstance();

        m_hubPose = FieldUtils.getInstance().getHubPose(DriverStation.getAlliance().get());

        m_cameraName = "Arducam_OV9782_D";
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

        List<VisionConfig> configs = cfg.getVisionConfigs();

        for (VisionConfig config : configs) {
            if (config.cameraName.equals(m_cameraName)) {
                m_cameraToTurret = new Transform3d(config.cameraTranslation, config.cameraRotation).inverse();
            }
        }
    }

    @Override
    public void execute() {
        Optional<VisionEstimationResult> result = m_Vision.getLatestFromCamera(m_cameraName);
        Pose3d turretPose;
        if (result.isPresent()) {
            Pose3d cameraPose = result.get().estimatedPose;
            turretPose = cameraPose.transformBy(m_cameraToTurret);
        } else {
            Pose3d chassisPose = new Pose3d(Drive.getInstance().getPose());
            turretPose = chassisPose.transformBy(m_chassisToTurret);
        }

        Translation2d off = m_hubPose.minus(turretPose).getTranslation().toTranslation2d();

        Rotation2d offAngle = off.getAngle();
        System.out.println("Got hub pose, angle: " + offAngle);

        m_Shooter.setTurretTarget(offAngle.getRadians(), 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {} 
}
