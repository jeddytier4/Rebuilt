package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Kinematics {
    public static final Translation3d HUB_POSITION_3D = new Translation3d(
            Units.inchesToMeters(182.11 - 5.0),
            Units.inchesToMeters(158.84),
            Units.inchesToMeters(72.00 + 8.0));

    public static final Translation2d HUB_POSITION_2D = HUB_POSITION_3D.toTranslation2d();
    public static final Transform2d SHOOTER_TRANSFORM = new Transform2d(0, 0, Rotation2d.k180deg);

    public static Transform2d getHubTransform2d(Pose2d pose, Translation2d hubPose) {
        Translation2d diff = hubPose.minus(pose.getTranslation());
        Rotation2d angle = diff.getAngle().minus(pose.getRotation());

        return new Transform2d(diff, angle);
    }

    public static Transform2d getHubTransform2d(Pose2d pose) {
        return getHubTransform2d(pose, HUB_POSITION_2D);
    }
}