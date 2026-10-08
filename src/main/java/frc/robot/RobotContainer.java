// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.camera.CameraSubsystem;
import frc.robot.climb.ClimbSubsystem;
import frc.robot.drive.AlignToPose;
import frc.robot.drive.DriveSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.HoodSubsystem;
import frc.robot.shooter.RollerSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.TurretSubsystem;
import frc.robot.utils.Kinematics;
import frc.robot.utils.TurretCalculator;
import frc.robot.utils.TurretCalculator.ShotData;
import frc.robot.utils.Utilites;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(turretSubsystem);
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final CameraSubsystem cameraSubsystem = new CameraSubsystem(driveSubsystem);
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    // private final CameraSubsystem cameraSubsystem = new
    // CameraSubsystem(driveSubsystem::getPose,
    // () -> driveSubsystem.gyro.getAngle(), () ->
    // Units.DegreesPerSecond.of(driveSubsystem.gyro.getRate()));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveSubsystem.controllerDriveRobotCentricCommand);
        shooterSubsystem.setDefaultCommand(shooterSubsystem.run(() -> {
            shooterSubsystem.setDutyCycle(0);
        }).withName("shooterDefault"));
        rollerSubsystem.setDefaultCommand(rollerSubsystem.run(() -> {
            rollerSubsystem.set(0);
        }).withName("rollerDefault"));
        intakeSubsystem
                .setDefaultCommand(intakeSubsystem.run(() -> intakeSubsystem.intake.set(0))
                        .withName("intakeDefault"));
        // hoodSubsystem.setDefaultCommand(hoodSubsystem.run(() ->
        // hoodSubsystem.setPos(MathUtil
        // .clamp(hoodSubsystem.primaryHoodServo.get() +
        // Constants.controller.getHoodDisplacement() * 0.05, 0,
        // 1)))
        // .withName("hoodDefault"));
        hoodSubsystem.setDefaultCommand(hoodSubsystem.run(() -> hoodSubsystem.setPos(MathUtil
                .clamp(hoodSubsystem.hoodSetpoint.getDouble(0), 0,
                        1)))
                .withName("hoodDefault"));
        turretSubsystem.setDefaultCommand(turretSubsystem.run(() -> {
            turretSubsystem.setPower(Constants.controller.getTurretDisplacement() * 0.2);
        }).withName("turretDefault"));
        climbSubsystem.setDefaultCommand(climbSubsystem.climbDefaultCommand());

        CommandScheduler.getInstance().registerSubsystem(cameraSubsystem);

        driveSubsystem.setLimelightPoseSupplier(cameraSubsystem::getPose);
        driveSubsystem.setLimelightTimestampSupplier(() -> cameraSubsystem.timestamp);
        driveSubsystem.setTurretAngleSupplier(() -> Units.Degrees.of(turretSubsystem.getHeading()));
        driveSubsystem.setLimelightUpdateSupplier(() -> !cameraSubsystem.rejectUpdate);
        driveSubsystem.setLimelightMt2Supplier(cameraSubsystem::useMegaTag2);

        Shuffleboard.getTab("Teleoperated").add("CS", CommandScheduler.getInstance());

        // CameraServer.startAutomaticCapture(0);

        // Configure the trigger bindings
        configureBindings();

        autoChooser.addOption("limlit", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> driveSubsystem.resetPos()),
                        driveSubsystem.run(() -> driveSubsystem.driveRobotCentric(-0.7, 0, 0)),
                        new RunCommand(() -> turretSubsystem.setTarget(0), turretSubsystem)),
                driveSubsystem.run(() -> driveSubsystem.driveRobotCentric(0, 0,
                        .5 * driveSubsystem.getMaxSpeed()))
                        .onlyWhile(() -> Double.isNaN(cameraSubsystem.getHubTagThetaDiff())),
                /*
                 * new WaitCommand(1.0)
                 * .raceWith(cameraSubsystem.useMt1Command()),
                 */
                new WaitCommand(1.0),
                new ParallelCommandGroup(
                        /*
                         * cameraSubsystem.useMt1Command().withDeadline(new WaitCommand(3.0)),
                         */
                        shooterSubsystem.run(
                                () -> shooterSubsystem.setVelocity(Units.RPM.of(Utilites
                                        .distanceToConfig(Units.Meters.of(
                                                Kinematics.HUB_POSITION_2D
                                                        .getDistance(driveSubsystem
                                                                .botToTurret(driveSubsystem
                                                                        .getPose())
                                                                .getTranslation()))).shooter_rpm))),
                        new SequentialCommandGroup(
                                new WaitCommand(.5),
                                new RepeatCommand(
                                        new WaitCommand(3).deadlineFor(
                                                rollerSubsystem.run(
                                                        () -> rollerSubsystem
                                                                .set(
                                                                        .5,
                                                                        Units.RPM.of(Utilites
                                                                                .distanceToConfig(
                                                                                        Units.Meters.of(
                                                                                                Kinematics.HUB_POSITION_2D
                                                                                                        .getDistance(
                                                                                                                driveSubsystem
                                                                                                                        .botToTurret(
                                                                                                                                driveSubsystem
                                                                                                                                        .getPose())
                                                                                                                        .getTranslation()))).kicker_rpm))))
                                                .andThen(
                                                        new WaitCommand(0.5)
                                                                .deadlineFor(
                                                                        rollerSubsystem.run(
                                                                                () -> rollerSubsystem
                                                                                        .set(
                                                                                                -.5,
                                                                                                Units.RPM.of(Utilites
                                                                                                        .distanceToConfig(
                                                                                                                Units.Meters
                                                                                                                        .of(Kinematics.HUB_POSITION_2D
                                                                                                                                .getDistance(
                                                                                                                                        driveSubsystem
                                                                                                                                                .botToTurret(
                                                                                                                                                        driveSubsystem
                                                                                                                                                                .getPose())
                                                                                                                                                .getTranslation()))).kicker_rpm))))))),
                        hoodSubsystem.run(() -> hoodSubsystem.setPos(Utilites
                                .distanceToConfig(Units.Meters.of(
                                        Kinematics.HUB_POSITION_2D.getDistance(
                                                driveSubsystem
                                                        .botToTurret(driveSubsystem
                                                                .getPose())
                                                        .getTranslation()))).servo_pos)),
                        faceHubCommand(),
                        intakeSubsystem.run(() -> intakeSubsystem.intake.set(.9)))));

        autoChooser.setDefaultOption("dumb auto", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> driveSubsystem
                                .resetPos(new Pose2d(Kinematics.HUB_POSITION_2D,
                                        Rotation2d.k180deg))),
                        driveSubsystem.run(() -> driveSubsystem.driveRobotCentric(0.7, 0, 0)),
                        new RunCommand(() -> turretSubsystem.setTarget(0), turretSubsystem)),
                // driveSubsystem.run(() -> driveSubsystem.driveRobotCentric(0, 0,
                // .5 * driveSubsystem.getMaxSpeed()))
                // .onlyWhile(() -> Double.isNaN(cameraSubsystem.getHubTagThetaDiff())),
                /*
                 * new WaitCommand(1.0)
                 * .raceWith(cameraSubsystem.useMt1Command()),
                 */
                new WaitCommand(1.0),
                new ParallelCommandGroup(
                        /*
                         * cameraSubsystem.useMt1Command().withDeadline(new WaitCommand(3.0)),
                         */
                        shooterSubsystem.run(
                                () -> shooterSubsystem.setVelocity(Units.RPM.of(4500))),
                        new SequentialCommandGroup(
                                new WaitCommand(.5),
                                rollerAndShuffleCommand()),
                        // hoodSubsystem.run(() -> hoodSubsystem.setPos(Utilites
                        // .distanceToConfig(Units.Meters.of(
                        // Kinematics.HUB_POSITION_2D.getDistance(
                        // driveSubsystem
                        // .botToTurret(driveSubsystem
                        // .getPose())
                        // .getTranslation()))).servo_pos)),
                        // faceHubCommand(),
                        intakeSubsystem.run(() -> intakeSubsystem.intake.set(.4)))));

        autoChooser.addOption("climb", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(2),
                                new InstantCommand(() -> driveSubsystem
                                        .resetPos(new Pose2d(Units.Inches
                                                .of(/* 76.5 */136),
                                                Units.Inches.of(/*
                                                                 * 150.5
                                                                 */76.5),
                                                Rotation2d.k180deg))),
                                driveSubsystem.run(() -> driveSubsystem
                                        .driveRobotCentric(0.7, 0, 0)),
                                new RunCommand(() -> turretSubsystem.setTarget(55),
                                        turretSubsystem))
                                .andThen(shootWithDistanceMapCommand()
                                        .alongWith(new WaitCommand(1)
                                                .andThen(rollerAndShuffleCommand().alongWith(
                                                        intakeSubsystem.setIntakeCommand(0.9))))
                                        .withTimeout(6)),
                        climbSubsystem.runToSetpointCommand(7.1)),
                rollerSubsystem.runOnce(() -> rollerSubsystem.set(0)),
                shooterSubsystem.runOnce(() -> shooterSubsystem.setDutyCycle(0)),
                // new AlignToPose(driveSubsystem, new Pose2d(Units.Inches.of(41.755 - 1),
                // driveSubsystem.getPose().getMeasureY(), Rotation2d.k180deg)).withDeadline(new
                // WaitCommand(2)),
                new AlignToPose(driveSubsystem, new Pose2d(Units.Inches.of(41.755 + 2.5),
                        Units.Inches.of(123.97 - 15.75 + 5), Rotation2d.k180deg), 1).raceWith(new WaitCommand(4)),
                Commands.run(() -> driveSubsystem.driveFieldCentric(0, 1, 0), driveSubsystem)
                        .withDeadline(new WaitCommand(1)),
                Commands.run(() -> driveSubsystem.driveFieldCentric(-0.5, 0, 0), driveSubsystem)
                        .withDeadline(new WaitCommand(1.5)),
                new InstantCommand(() -> {
                    System.out.println("align end");
                }),
                driveSubsystem.run(() -> driveSubsystem.driveRobotCentric(0, -0.4, 0)).withDeadline(
                        new WaitCommand(1).andThen(climbSubsystem.runToSetpointCommand(2.0)))));

        // XboxController asdqwe = new XboxController(1);

        // new Trigger(asdqwe::getAButton).whileTrue(new StartEndCommand(
        // () -> asdqwe.setRumble(RumbleType.kBothRumble, 1),
        // () -> asdqwe.setRumble(RumbleType.kBothRumble, 0)));

        Shuffleboard.getTab("Teleoperated").add(autoChooser);
        Shuffleboard.getTab("Climb").add(new AlignToPose(driveSubsystem,
                new Pose2d(Units.Inches.of(41.755 - 1), Units.Inches.of(123.97 - 15.75 / 2),
                        Rotation2d.k180deg)));
    }

    private Command rollerAndShuffleCommand() {
        return new RepeatCommand(
                new WaitCommand(3).deadlineFor(
                        rollerSubsystem.run(
                                () -> rollerSubsystem
                                        .set(
                                                .5,
                                                Units.RPM.of(3000))))
                        .andThen(
                                new WaitCommand(0.5)
                                        .deadlineFor(
                                                rollerSubsystem.run(
                                                        () -> rollerSubsystem
                                                                .set(
                                                                        -.5,
                                                                        Units.RPM
                                                                                .of(3000))))));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        new Trigger(() -> Constants.controller.getResetPoseButton())
                .onTrue(Commands.runOnce(() -> driveSubsystem.resetPos()));
        // .whileTrue(driveSubsystem.run(() -> driveSubsystem.voltageDrive()));

        // Drive field centric
        new Trigger(() -> Constants.controller.getDriveFieldCentricMode())
                .toggleOnTrue(driveSubsystem.controllerDriveFieldCentricFlickStickCommand());

        // // Drive field centric facing origin
        // new Trigger(() ->
        // Constants.controller.getDriveFieldCentricFacingOriginMode())
        // .whileTrue(driveSubsystem.controllerDriveFieldCentricFacingPoseCommand(() ->
        // 0, () -> 0));

        new Trigger(Constants.controller::allShoot).whileTrue(shootWithDistanceMapCommand());

        // new Trigger(() ->
        // DriverStation.isEnabled()).whileTrue(shooterSubsystem.runEnd(shooterSubsystem.orchestra::play,
        // shooterSubsystem.orchestra::stop));

        // // Drive field centric snapping
        // new Trigger(() -> Constants.controller.getDriveFieldCentricSnappingMode())
        // .whileTrue(driveSubsystem.controllerDriveFieldCentricSnapCommand());

        // // Reset pose to origin
        // new Trigger(() -> Constants.controller.getResetPoseButton())
        // .onTrue(Commands.runOnce(driveSubsystem::setPoseToCam));

        // // Align to origin pose with deceleration
        // new Trigger(() -> Constants.controller.getAlignToOriginPoseButton())
        // .whileTrue(new AlignToPose(driveSubsystem, new Pose2d(0, 0,
        // Rotation2d.kZero)));

        // new Trigger(() ->
        // Constants.controller.getResetPoseButton()).whileTrue(turretSubsystem.run(()
        // -> {
        // double theta = cameraSubsystem.getThetaDiff();
        // turretSubsystem.setPower(-(theta / 180 * .5));
        // System.out.println(theta);
        // }));

        new Trigger(() -> Constants.controller.getShoot()).whileTrue(shooterSubsystem.runEnd(() -> {
            shooterSubsystem.setVelocity(Units.RPM.of(shooterSubsystem.targetRPM.getDouble(3000)));

        }, () -> shooterSubsystem.setDutyCycle(0)));

        new Trigger(() -> Constants.controller.getRoller()).whileTrue(
                rollerSubsystem.runEnd(() -> {
                    rollerSubsystem.set(.5,
                            Units.RPM.of(rollerSubsystem.kickerSpeed.getDouble(3000)));
                }, () -> rollerSubsystem.set(0)));

        new Trigger(() -> Constants.controller.getIntake()).whileTrue(
                intakeSubsystem.runEnd(() -> intakeSubsystem.intake.set(.9),
                        () -> intakeSubsystem.intake.set(0)));

        new Trigger(() -> Constants.controller.getDriveFieldCentricFacingHubMode()).whileTrue(faceHubCommand());

        new Trigger(Constants.controller::climbUp).whileTrue(climbSubsystem.setPowerCommand(0.5));

        new Trigger(Constants.controller::climbDown).whileTrue(climbSubsystem.setPowerCommand(-0.5));
    }

    public GenericEntry flightTimeEntry = Shuffleboard.getTab("Teleoperated").add("Flight Time", 0).getEntry();

    public Command faceHubCommand() {
        return Commands.run(() -> {
            double angle = Kinematics
                    .getHubTransform2d(driveSubsystem
                            .botToTurret(driveSubsystem.getFutureRobotPose2d()))
                    .getRotation().getDegrees();

            // double distance = TurretCalculator
            // .getDistanceToTarget(driveSubsystem.getTurretPose(),
            // Constants.FieldConstants.HUB_BLUE)
            // .in(Units.Meters);

            // ShotData shot = Constants.SHOT_MAP.get(distance);

            // ShotData shot = TurretCalculator.iterativeMovingShotFromMap(
            // driveSubsystem.getTurretPose(),
            // driveSubsystem.getFieldVelocity(),
            // Constants.FieldConstants.HUB_BLUE,
            // 5);

            // Angle azimuthAngle =
            // TurretCalculator.calculateAzimuthAngle(driveSubsystem.getPose(),
            // shot.getTarget());

            // double angle =
            // driveSubsystem.getPredictedHubTransform2d(flightTimeEntry.getDouble(0))
            // .getRotation()
            // .getDegrees();

            // double angle = azimuthAngle.in(Degrees);
            angle = MathUtil.clamp(angle, Constants.TURRET_MIN_ANGLE.in(Degrees),
                    Constants.TURRET_MAX_ANGLE.in(Degrees));

            turretSubsystem.setTarget(angle);
        }, turretSubsystem);
    }

    public void shootWithDistanceMap() {
        // double distance = TurretCalculator
        // .getDistanceToTarget(driveSubsystem.getTurretPose(),
        // Constants.FieldConstants.HUB_BLUE)
        // .in(Units.Meters);

        // ShotData shot = Constants.SHOT_MAP.get(distance);

        ShotData shot = TurretCalculator.iterativeMovingShotFromMap(
                driveSubsystem.getTurretPose(),
                driveSubsystem.getFieldVelocity(),
                Constants.FieldConstants.HUB_BLUE,
                5);

        Pose2d thegoodposemaybe = new Pose2d(shot.getTarget().toTranslation2d(), Rotation2d.kZero);

        driveSubsystem.field.getObject("predicted-hub").setPose(thegoodposemaybe);

        Angle azimuthAngle = TurretCalculator.calculateAzimuthAngle(driveSubsystem.getPose(), shot.getTarget());

        double angle = MathUtil.clamp(azimuthAngle.in(Degrees), Constants.TURRET_MIN_ANGLE.in(Degrees),
                Constants.TURRET_MAX_ANGLE.in(Degrees));
        turretSubsystem.setTarget(angle);

        hoodSubsystem.setPos(shot.hoodAngle());
        shooterSubsystem.setVelocity(shot.getAngularExitVelocity());
    }

    public Command shootWithDistanceMapCommand() {
        return new RunCommand(this::shootWithDistanceMap, shooterSubsystem, hoodSubsystem, turretSubsystem);
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return driveSubsystem.driveRobotCentricCommand(() -> 1.0, () -> 0.0, () ->
        // 0.0)
        // .withDeadline(Commands.waitSeconds(1));
        return autoChooser.getSelected();
        // return Autos.exampleAuto(m_driveSubsystem);
    }
}
