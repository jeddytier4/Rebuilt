package frc.robot.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AlignToPose extends Command {
    public Pose2d targetPose;
    public PIDController xController, yController, rotController;

    private Timer alignedTimer;

    private DriveSubsystem driveSubsystem;

    private double maxOutput;

    public AlignToPose(DriveSubsystem driveSubsystem, Pose2d targetPose, double maxOutput) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        xController = new PIDController(5, 0.01, 0.3);
        yController = new PIDController(8.5, 0.01, 0.2);
        rotController = new PIDController(0.04, 0.0001, 0.001);
        rotController.enableContinuousInput(-180.0, 180.0);

        xController.setTolerance(Units.inchesToMeters(2.5));
        yController.setTolerance(Units.inchesToMeters(2.5));
        rotController.setTolerance(3);

        alignedTimer = new Timer();

        this.driveSubsystem = driveSubsystem;
        this.maxOutput = maxOutput;

        addRequirements(driveSubsystem);

        this.setName("AlignToPose " + targetPose);
    }

    public AlignToPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this(driveSubsystem, targetPose, Double.MAX_VALUE);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rotController.setSetpoint(targetPose.getRotation().getDegrees());

        xController.reset();
        yController.reset();
        rotController.reset();

        alignedTimer.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double rotSpeed = rotController.calculate(currentPose.getRotation().getDegrees());
 
        // xSpeed = MathUtil.clamp(xSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
        // ySpeed = MathUtil.clamp(ySpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
        
        double xError = xController.getError();
        double yError = yController.getError();
        double dist = Math.sqrt(xError * xError + yError * yError);

        double deceleration = 2.0; // m/s^2

        double maxSpeed = Math.sqrt(2 * deceleration * dist); // add a small constant to prevent stalling
        maxSpeed = MathUtil.clamp(maxSpeed, 0.2, Constants.DRIVE_MAX_SPEED);
        if (DriverStation.isTest()) {
            SmartDashboard.putNumber("align_rawXSpeed", xSpeed);
            SmartDashboard.putNumber("align_rawYSpeed", ySpeed);
            SmartDashboard.putNumber("align_rawRotSpeed", rotSpeed);
        }

        double div = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        if (div > maxSpeed) {
            xSpeed = xSpeed / div * maxSpeed;
            ySpeed = ySpeed / div * maxSpeed;
        }

        rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
        xSpeed = MathUtil.clamp(xSpeed, -maxOutput, maxOutput);
        ySpeed = MathUtil.clamp(ySpeed, -maxOutput, maxOutput);

        driveSubsystem.driveFieldCentric(xSpeed, ySpeed, rotSpeed);

        if (DriverStation.isTest()) {
            SmartDashboard.putNumber("align_clampedXSpeed", xSpeed);
            SmartDashboard.putNumber("align_clampedYSpeed", ySpeed);
            SmartDashboard.putNumber("align_clampedRotSpeed", rotSpeed);
            SmartDashboard.putNumber("align_currentX", currentPose.getX());
            SmartDashboard.putNumber("align_currentY", currentPose.getY());
            SmartDashboard.putNumber("align_currentRot", currentPose.getRotation().getDegrees());
            SmartDashboard.putNumber("align_targetX", targetPose.getX());
            SmartDashboard.putNumber("align_targetY", targetPose.getY());
            SmartDashboard.putNumber("align_targetRot", targetPose.getRotation().getDegrees());
            SmartDashboard.putNumber("align_dist", dist);
            SmartDashboard.putNumber("align_maxSpeed", maxSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public boolean isAligned() {
        // System.out.println("x: " + xController.atSetpoint());
        // System.out.println("y: " + yController.atSetpoint() + "err: " + yController.getError());
        // System.out.println("rot: " + rotController.atSetpoint());
        return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        if (!alignedTimer.isRunning() && isAligned()) {
            alignedTimer.restart();
        } else if (alignedTimer.isRunning() && !isAligned()) {
            alignedTimer.stop();
        }

        return isAligned() && alignedTimer.hasElapsed(0.2);
    }

}
