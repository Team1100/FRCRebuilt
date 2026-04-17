package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.drive.TargetPose;

public class DriveToTDPose extends Command {
    private TDNumber m_targetX;
    private TDNumber m_targetY;
    private TDNumber m_targetTheta;

    private Drive m_Drive;
    private DriveToPose m_DriveToPose;

    public DriveToTDPose() {
        super(Drive.getInstance(), "Debug", "DriveToTDPose");
        m_Drive = Drive.getInstance();
        m_DriveToPose = new DriveToPose(this::getTargetPose, m_Drive::getPose);

        m_targetX = new TDNumber(m_Drive, "Debug", "TargetX");
        m_targetY = new TDNumber(m_Drive, "Debug", "TargetY");
        m_targetTheta = new TDNumber(m_Drive, "Debug", "TargetTheta");
    }

    public TargetPose getTargetPose() {
        return new TargetPose(
            new Pose2d(
                new Translation2d(m_targetX.get(), m_targetY.get()),
                new Rotation2d(m_targetTheta.get())));
    }

    @Override
    public void initialize() {
        m_Drive.resetOdometry(new Pose2d());
        m_targetX.set(m_Drive.getPose().getX());
        m_targetY.set(m_Drive.getPose().getY());
        m_targetTheta.set(m_Drive.getHeading());
        m_DriveToPose.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveToPose.cancel();
    }
}
