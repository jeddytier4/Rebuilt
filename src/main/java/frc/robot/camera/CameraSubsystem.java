// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.camera;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class CameraSubsystem extends SubsystemBase {
    public HttpCamera turretLimelight;
    // public Supplier<Pose2d> poseSupplier;
    // public DoubleSupplier yaw;
    // public Supplier<AngularVelocity> angularVelocity;
    public Supplier<Angle> robotYaw;
    public boolean doPoseEstimation;
    public Pose2d limelightPose;
    public Pose2d robotPose;
    public double timestamp;
    public boolean rejectUpdate;
    public GenericEntry megatag2;

    private DriveSubsystem driveSubsystem;
    // private TurretSubsystem turretSubsystem;

    /** Creates a new ExampleSubsystem. */
    public CameraSubsystem(DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;
        // this.poseSupplier = poseSupplier;
        // this.yaw = yawSupplier;
        // this.angularVelocity = angularVelocitySupplier;
        this.limelightPose = Pose2d.kZero;
        this.timestamp = 0;
        this.rejectUpdate = true;
        this.doPoseEstimation = true;
        Shuffleboard.getTab("camera").addBoolean("megatag2", this::useMegaTag2);
        if (DriverStation.isTest())
            Shuffleboard.getTab("camera").addDouble("tx", () -> getHubTagThetaDiff());
        // turretLimelight = new HttpCamera("turret_limelight", "10.52.43.11:5800");

        // Shuffleboard.getTab("camera").add(turretLimelight);
    }

    public void updateVisionPose() {

        // boolean useMegaTag2 = megatag2.getBoolean(true); // set to false to use
        // MegaTag1
        boolean doRejectUpdate = false;
        if (useMegaTag2() == false) {
            LimelightHelpers.PoseEstimate mt1;
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
                mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
            else
                mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                limelightPose = mt1.pose;
                timestamp = mt1.timestampSeconds;
            }
            this.rejectUpdate = doRejectUpdate;
        } else if (useMegaTag2() == true) {
            // double angle = driveSubsystem.getPose().getRotation().getDegrees() + 180;
            double angle = driveSubsystem.getEstimatedCameraPose().getRotation().getDegrees();
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
                angle += 180;

            LimelightHelpers.SetRobotOrientation("limelight",
                    angle, 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2;
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
                mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
            else
                mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            // if (Math.abs(angularVelocity.get().in(Units.DegreesPerSecond)) > 720) // if
            // our angular velocity is greater
            // // than 720 degrees per second,
            // ignore vision updates
            // {
            // doRejectUpdate = true;
            // }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                limelightPose = mt2.pose;
                timestamp = mt2.timestampSeconds;
            }
            this.rejectUpdate = doRejectUpdate;
        }
    }

    public Pose2d getPose() {
        return limelightPose;
    }

    /**
     * Returns the theta difference (tx value) between the limelight and the current april tag, assuming said april tag is on the hub.
     * 
     * @return theta if looking at hub tag, else Double.NaN
     */
    public double getHubTagThetaDiff() {
        Double[] validIDs = { 11.0, 2.0, 3.0, 4.0, 5.0, 8.0, 9.0, 10.0, 19.0, 20.0, 21.0, 24.0, 25.0, 26.0, 18.0,
                27.0 };

        if (Arrays.asList(validIDs).contains(LimelightHelpers.getFiducialID(Constants.TURRET_LIMELIGHT)))
            return LimelightHelpers.getTX(Constants.TURRET_LIMELIGHT);
        return Double.NaN;
    }

    // Only use MegaTag1 when back button is held
    public boolean useMegaTag2() {
        return !mt1Override && !Constants.controller.getResetPoseButton();
    }

    public boolean mt1Override = false;

    public Command useMt1Command() {
        return startEnd(() -> mt1Override = true, () -> mt1Override = false);
    }

    @Override
    public void periodic() {
        if (this.doPoseEstimation) {
            updateVisionPose();
        }
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
