package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestSubsystem extends SubsystemBase {

    private QuestNav questNav;
    private DriveSubsystem driveSubsystem;

    public QuestSubsystem(DriveSubsystem driveSubsystem) {
        questNav = new QuestNav();
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(Constants.ROBOT_TO_QUEST.inverse());

                // You can put some sort of filtering here if you would like!

                // Add the measurement to our estimator
                driveSubsystem.swerveDrive.addVisionMeasurement(robotPose.toPose2d(), timestamp, VecBuilder.fill(
                        0.02, // X position trust (20 mm)
                        0.02, // Y position trust (20 mm)
                        0.0872665)); // Rotation trust (5 degrees));
                // swerveDriveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp,
                // QUESTNAV_STD_DEVS);
            }
        }

        SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
        SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
        SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
        questNav.getBatteryPercent().ifPresent(
                b -> SmartDashboard.putNumber("QuestNav/Battery%", b));
        questNav.getTrackingLostCounter().ifPresent(
                c -> SmartDashboard.putNumber("QuestNav/TrackingLostCount", c));
    }
}
