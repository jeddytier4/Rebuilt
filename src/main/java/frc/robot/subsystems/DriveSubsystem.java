// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    public SwerveDrive swerveDrive;
    public Field2d field;

    public DriveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory)
                    .createSwerveDrive(Constants.MAX_SPEED.in(Units.MetersPerSecond));
        } catch (IOException e) {
            DriverStation.reportError("ur prolly pretty dumb if u hit this lol", e.getStackTrace());
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));

        ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
        field = new Field2d();
        tab.add(field);

        // swerveDrive.setMaximumAllowableSpeeds(3, 0);
        System.out.println(swerveDrive.getMaximumChassisVelocity());
        System.out.println(swerveDrive.getMaximumChassisAngularVelocity());

    }

    public boolean isStill() {
        double x = swerveDrive.getFieldVelocity().vxMetersPerSecond;
        double y = swerveDrive.getFieldVelocity().vyMetersPerSecond;
        double rot = swerveDrive.getFieldVelocity().omegaRadiansPerSecond;
        return Math.abs(x) < 0.02 && Math.abs(y) < 0.02 && rot < 0.1;
    }

    public GenericEntry useHeadingToggle = Shuffleboard.getTab("Swerve")
            .add("Use heading (warning: turns suddenly)", false).getEntry();

    public Command controllerDriveCommand() {
        return run(() -> {
            double x = -Constants.controller.getLeftY();
            double y = -Constants.controller.getLeftX();

            double deadband = 0.1;
            double driveSpeed = 0.3 + 0.5 * Constants.controller.getLeftTriggerAxis();

            if (x * x + y * y < deadband * deadband) {
                x = 0;
                y = 0;
            } else {
                double magnitude = Math.sqrt(x * x + y * y);
                double angle = Math.atan2(y, x);
                magnitude = (magnitude - deadband) / (1 - deadband);
                x = magnitude * magnitude * Math.cos(angle);
                y = magnitude * magnitude * Math.sin(angle);
            }

            ChassisSpeeds tmp;

            boolean useHeading = useHeadingToggle.getBoolean(false);

            if (useHeading) {
                tmp = swerveDrive.swerveController.getTargetSpeeds(
                        x,
                        y,
                        -Constants.controller.getRightX(),
                        -Constants.controller.getRightY(),
                        swerveDrive.getOdometryHeading().getRadians(),
                        driveSpeed * swerveDrive.getMaximumChassisAngularVelocity());
            } else {
                double xMetersPerSecond = x * driveSpeed * swerveDrive.getMaximumChassisAngularVelocity();
                double yMetersPerSecond = y * driveSpeed * swerveDrive.getMaximumChassisAngularVelocity();
                double omegaRadiansPerSecond = -Constants.controller.getRightX() * driveSpeed
                        * swerveDrive.getMaximumChassisAngularVelocity();
                tmp = swerveDrive.swerveController.getRawTargetSpeeds(
                        xMetersPerSecond,
                        yMetersPerSecond,
                        omegaRadiansPerSecond);
            }
            swerveDrive.driveFieldOriented(tmp);
            System.out.println("odom=%.3f rightX=%.3f rightY=%.3f targetOmega=%.3f".formatted(
                    swerveDrive.getOdometryHeading().getRadians(),
                    -Constants.controller.getRightX(),
                    -Constants.controller.getRightY(), tmp.omegaRadiansPerSecond));
            // swerveDrive.driveFieldOriented(new ChassisSpeeds(translation.getX(),
            // translation.getY(),
            // .5 * swerveDrive.getMaximumChassisAngularVelocity()
            // * SwerveMath.applyDeadband(-Constants.controller.getRightX(), true, 0.25)));
            // System.out.println(swerveDrive.getMaximumChassisAngularVelocity());
        });
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        field.setRobotPose(swerveDrive.getPose());
        // swerveDrive.updateOdometry();

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
