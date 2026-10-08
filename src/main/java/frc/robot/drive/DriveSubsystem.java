// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.CustomMecanumDrive;
import frc.lib.CustomMecanumDriveKinematics;
import frc.robot.Constants;
import frc.robot.shooter.TurretSubsystem;
import frc.robot.utils.Kinematics;
import frc.robot.utils.TurretCalculator;

public class DriveSubsystem extends SubsystemBase {

  public CANVenom flMotor;
  public CANVenom frMotor;
  public CANVenom blMotor;
  public CANVenom brMotor;

  public Encoder flEncoder;
  public Encoder frEncoder;
  public Encoder blEncoder;
  public Encoder brEncoder;

  public SimpleMotorFeedforward flFeedforward;
  public SimpleMotorFeedforward frFeedforward;
  public SimpleMotorFeedforward blFeedforward;
  public SimpleMotorFeedforward brFeedforward;

  public PIDController flPID;
  public PIDController frPID;
  public PIDController blPID;
  public PIDController brPID;

  public CustomMecanumDrive drive;
  public MecanumDriveKinematics kinematics;
  public MecanumDrivePoseEstimator poseEstimator;

  public AHRS gyro;

  public SysIdRoutine sysId;

  public Field2d field;

  public Supplier<Pose2d> limelightPoseSupplier;
  public DoubleSupplier limelightTimestampSupplier;
  public Supplier<Angle> turretAngle;
  public Supplier<Boolean> doUpdate;
  public BooleanSupplier limelightMt2Supplier;

  private PIDController rotController = new PIDController(0.04, 0.0, 0.0);

  private Rotation2d snapDirection = Rotation2d.kZero;

  public GenericEntry volts;
  private TurretSubsystem turretSubsystem;

  boolean isUsingFlickStick = false;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    flMotor = new CANVenom(Constants.FL_MOTOR);
    frMotor = new CANVenom(Constants.FR_MOTOR);
    blMotor = new CANVenom(Constants.BL_MOTOR);
    brMotor = new CANVenom(Constants.BR_MOTOR);

    flMotor.setInverted(true);
    frMotor.setInverted(false);
    blMotor.setInverted(true);
    brMotor.setInverted(false);

    flMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    frMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    blMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    brMotor.setBrakeCoastMode(BrakeCoastMode.Brake);

    double p = 0.3, i = 0.0, d = 0.03, f = 0.184, b = 0.0;
    // 0.195, 0, 0.01, 0.184, 0
    flMotor.setPID(p, i, d, f, b);
    frMotor.setPID(p, i, d, f, b);
    blMotor.setPID(p, i, d, f, b);
    brMotor.setPID(p, i, d, f, b);

    flFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);
    frFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);
    blFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);
    brFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);

    flPID = new PIDController(Constants.DRIVE_kP, 0, 0);
    frPID = new PIDController(Constants.DRIVE_kP, 0, 0);
    blPID = new PIDController(Constants.DRIVE_kP, 0, 0);
    brPID = new PIDController(Constants.DRIVE_kP, 0, 0);

    flMotor.setMaxSpeed(5000);
    frMotor.setMaxSpeed(5000);
    blMotor.setMaxSpeed(5000);
    brMotor.setMaxSpeed(5000);

    double maxAcceleration = Constants.DRIVE_MAX_ACCELERATION / Constants.WHEEL_DISTANCE_PER_MOTOR_REV * 60;
    flMotor.setMaxAcceleration(maxAcceleration);
    frMotor.setMaxAcceleration(maxAcceleration);
    blMotor.setMaxAcceleration(maxAcceleration);
    brMotor.setMaxAcceleration(maxAcceleration);

    double maxJerk = 10.0 * maxAcceleration;
    flMotor.setMaxJerk(maxJerk);
    frMotor.setMaxJerk(maxJerk);
    blMotor.setMaxJerk(maxJerk);
    brMotor.setMaxJerk(maxJerk);

    System.out.println(flMotor.getMaxSpeed());

    flEncoder = new Encoder(Constants.FL_ENCODER[0], Constants.FL_ENCODER[1], true);
    frEncoder = new Encoder(Constants.FR_ENCODER[0], Constants.FR_ENCODER[1], false);
    blEncoder = new Encoder(Constants.BL_ENCODER[0], Constants.BL_ENCODER[1], true);
    brEncoder = new Encoder(Constants.BR_ENCODER[0], Constants.BR_ENCODER[1], false);

    // this makes the encoder return the wheels linear distance and velocity in
    // meters and meters per second
    flEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);
    frEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);
    blEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);
    brEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);

    gyro = new AHRS(NavXComType.kMXP_SPI);
    // if (DriverStation.getAlliance().isPresent() &&
    // DriverStation.getAlliance().get() == Alliance.Red)
    // gyro.setAngleAdjustment(180);
    gyro.reset();

    // ControlMode controlMode = ControlMode.SpeedControl;
    /*** m/s to rpm */
    // double metersPerSecondToRPM = 60 / Constants.WHEEL_DISTANCE_PER_MOTOR_REV;
    drive = new CustomMecanumDrive(
        v -> flMotor.setVoltage(flFeedforward.calculate(v) + flPID.calculate(flEncoder.getRate(), v)),
        v -> blMotor.setVoltage(blFeedforward.calculate(v) + blPID.calculate(blEncoder.getRate(), v)),
        v -> frMotor.setVoltage(frFeedforward.calculate(v) + frPID.calculate(frEncoder.getRate(), v)),
        v -> brMotor.setVoltage(brFeedforward.calculate(v) + brPID.calculate(brEncoder.getRate(), v)));
    // v -> flMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1,
    // 1)),
    // v -> blMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1,
    // 1)),
    // v -> frMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1,
    // 1)),
    // v -> brMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1,
    // 1)));

    // TODO: get measurements of wheels
    kinematics = new CustomMecanumDriveKinematics(
        new Translation2d(Units.Meters.of(0.2585708316), Units.Meters.of(-0.25691592)), // front left
        new Translation2d(Units.Meters.of(0.2585708316), Units.Meters.of(0.25691592)), // front right
        new Translation2d(Units.Meters.of(-0.2585708316), Units.Meters.of(-0.25691592)), // back left
        new Translation2d(Units.Meters.of(-0.2585708316), Units.Meters.of(0.25691592)), // back right
        131.0 / 133.0,
        138.0 / 161.0);

    poseEstimator = new MecanumDrivePoseEstimator(kinematics, new Rotation2d(-gyro.getYaw()),
        new MecanumDriveWheelPositions(), Pose2d.kZero);

    this.sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
        voltage -> {
          flMotor.setVoltage(voltage.magnitude());
          frMotor.setVoltage(voltage.magnitude());
          blMotor.setVoltage(voltage.magnitude());
          brMotor.setVoltage(voltage.magnitude());
        },
        log -> {
          log.motor("drive-front-right")
              .voltage(Units.Volts.of(frMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(-frEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-frEncoder.getRate()));

          log.motor("drive-front-left")
              .voltage(Units.Volts.of(flMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(-flEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-flEncoder.getRate()));

          log.motor("drive-back-left")
              .voltage(Units.Volts.of(blMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(-blEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-blEncoder.getRate()));

          log.motor("drive-back-right")
              .voltage(Units.Volts.of(brMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(-brEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-brEncoder.getRate()));
        },
        this));

    // if (DriverStation.isTest()) {
    tab.addDouble("gyroYaw", () -> gyro.getAngle());
    tab.add("flMotor", flMotor);
    tab.add("frMotor", frMotor);
    tab.add("blMotor", blMotor);
    tab.add("brMotor", brMotor);

    tab.add(drive);
    tab.addDouble("flMotorTarget", () -> flMotor.getPIDTarget());
    tab.addDouble("frMotorTarget", () -> frMotor.getPIDTarget());
    tab.addDouble("blMotorTarget", () -> blMotor.getPIDTarget());
    tab.addDouble("brMotorTarget", () -> brMotor.getPIDTarget());

    ShuffleboardTab othertab = Shuffleboard.getTab("more drivetrain");
    othertab.addDouble("flMotorHBridge", () -> flMotor.get());
    othertab.addDouble("frMotorHBridge", () -> frMotor.get());
    othertab.addDouble("blMotorHBridge", () -> blMotor.get());
    othertab.addDouble("brMotorHBridge", () -> brMotor.get());
    // ShuffleboardLayout encoderLayout =
    // tab.getLayout("encoders", "kList");

    tab.add("flEncoder", flEncoder);
    tab.add("frEncoder", frEncoder);
    tab.add("blEncoder", blEncoder);
    tab.add("brEncoder", brEncoder);
    tab.addDouble("poseX", () -> getPose().getMeasureX().in(Units.Inches));
    tab.addDouble("poseY", () -> getPose().getMeasureY().in(Units.Inches));
    tab.addDouble("poseYaw", () -> getPose().getRotation().getDegrees());
    tab.addDouble("feedforward-res", () -> flFeedforward.calculateWithVelocities(0, 2));
    
    tab.addBoolean("Field Centric Enabled", () -> isUsingFlickStick);

    tab.addDouble("dist_to_hub",
        () -> (Kinematics.HUB_POSITION_2D.getDistance(this.botToTurret(this.getPose()).getTranslation())));
    // }

    tab.addDoubleArray("gyro", () -> {
      double[] dub = { gyro.getYaw(), gyro.getPitch(), gyro.getRoll() };
      return dub;
    });

    tab.addDouble("Shiftt Time Remains", () -> remainingHubSwitchTime());
    tab.addBoolean("Hub active", () -> isHubActive());

    tab.addDouble("Meters to Hub",
        () -> (TurretCalculator.getDistanceToTarget(getTurretPose(), Constants.FieldConstants.HUB_BLUE))
            .in(Units.Meters));

    tab.addDouble("Velocity X", () -> getVelocity().vxMetersPerSecond);
    tab.addDouble("Velocity Y", () -> getVelocity().vyMetersPerSecond);
    tab.addDouble("Velocity deg", () -> Math.toDegrees(getVelocity().omegaRadiansPerSecond));

    field = new Field2d();
    tab.add("Field", field);

    rotController.enableContinuousInput(-180, 180);
  }

  public void setLimelightPoseSupplier(Supplier<Pose2d> supllier) {
    limelightPoseSupplier = supllier;
  }

  public void setLimelightTimestampSupplier(DoubleSupplier supplier) {
    limelightTimestampSupplier = supplier;
  }

  public void setTurretAngleSupplier(Supplier<Angle> supplier) {
    turretAngle = supplier;
  }

  public void setLimelightUpdateSupplier(Supplier<Boolean> supplier) {
    doUpdate = supplier;
  }

  public void setLimelightMt2Supplier(BooleanSupplier limelightMt2Supplier) {
    this.limelightMt2Supplier = limelightMt2Supplier;
  }

  public void setPoseToCam() {
    Pose2d robotPose = cameraToBot(limelightPoseSupplier.get());
    poseEstimator.resetPose(new Pose2d(robotPose.getMeasureX(), robotPose.getMeasureY(), gyro.getRotation2d()));
  }

  public double getMaxSpeed() {
    double speed = Constants.DRIVE_MAX_SPEED;
    speed += 2 * Constants.controller.getDriveBoost();

    return speed;
  }

  /**
   * @param xSpeed    meters per second (+ forward)
   * @param ySpeed    meters per second (+ left)
   * @param zRotation i don't know what unit this is (: (+ counterclockwise)
   */
  public void driveRobotCentric(double xSpeed, double ySpeed, double zRotation) {
    drive.driveCartesian(-xSpeed, ySpeed, zRotation);
  }

  /**
   * @param xSpeed    meters per second (+ forward)
   * @param ySpeed    meters per second (+ left)
   * @param zRotation i don't know what unit this is (: (+ counterclockwise)
   */
  public void driveFieldCentric(double xSpeed, double ySpeed, double zRotation) {
    // double rot = getPose().getRotation;
    Translation2d t = new Translation2d(xSpeed, ySpeed).rotateBy(getPose().getRotation().times(-1));
    xSpeed = t.getX();
    ySpeed = t.getY() * 1.5;
    drive.driveCartesian(-xSpeed, ySpeed, zRotation);
  }

  public Command driveFieldCentricCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation) {
    return run(
        () -> driveFieldCentric(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zRotation.getAsDouble()))
        .withName("driveFieldCentric");
  }

  public void controllerDriveFieldCentric() {
    double driveX = Constants.controller.getDriveX();
    double driveY = -Constants.controller.getDriveY();
    double driveTurn = -Constants.controller.getDriveTurn();

    driveX = Math.signum(driveX) * driveX * driveX;
    driveY = Math.signum(driveY) * driveY * driveY;
    driveTurn = Math.signum(driveTurn) * driveTurn * driveTurn;

    double speed = getMaxSpeed();

    driveX *= speed;
    driveY *= speed;
    driveTurn *= speed;

    driveFieldCentric(driveX, driveY, driveTurn);
  }

  public Command controllerDriveFieldCentricCommand = run(this::controllerDriveFieldCentric)
      .withName("driveControllerFieldCentric");

  public void controllerDriveFieldCentricFacingDir(Rotation2d dir, double turnStrength) {
    double driveX = Constants.controller.getDriveX();
    double driveY = -Constants.controller.getDriveY();

    driveX = Math.signum(driveX) * driveX * driveX;
    driveY = Math.signum(driveY) * driveY * driveY;

    double speed = getMaxSpeed();

    driveX *= speed;
    driveY *= speed;

    double rotSpeed = 0;

    double angle = dir.getDegrees();

    rotController.setSetpoint(angle);
    rotSpeed = rotController.calculate(getPose().getRotation().getDegrees());

    rotSpeed *= turnStrength;

    rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
    // }

    driveFieldCentric(driveX, driveY, rotSpeed);
  }

  public void controllerDriveFieldCentricFacingDir(Rotation2d dir) {
    controllerDriveFieldCentricFacingDir(dir, 1.0);
  }

  public void controllerDriveFieldCentricFacingPose(double x, double y) {
    Rotation2d dir = getPose().getRotation();

    Translation2d trans = new Translation2d(x, y)
        .minus(getPose().getTranslation());
    if (trans.getNorm() > 0.1) {
      dir = trans.getAngle();
    } // Don't auto rotate when less than 10cm away from target

    controllerDriveFieldCentricFacingDir(dir);
  }

  public void controllerDriveRobotCentricFacingDir(Rotation2d dir, double turnStrength) {
    double driveX = Constants.controller.getDriveX();
    double driveY = -Constants.controller.getDriveY();

    driveX = Math.signum(driveX) * driveX * driveX;
    driveY = Math.signum(driveY) * driveY * driveY;

    double speed = getMaxSpeed();

    driveX *= speed;
    driveY *= speed;

    double rotSpeed = 0;

    double angle = dir.getDegrees();

    rotController.setSetpoint(angle);
    rotSpeed = rotController.calculate(getPose().getRotation().getDegrees());

    rotSpeed *= turnStrength;

    rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
    // }

    driveRobotCentric(driveX, driveY, rotSpeed);
  }

  public void controllerDriveRobotCentricFacingDir(Rotation2d dir) {
    controllerDriveRobotCentricFacingDir(dir, 1.0);
  }

  public Command controllerDriveFieldCentricFacingPoseCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return run(() -> controllerDriveFieldCentricFacingPose(xSupplier.getAsDouble(), ySupplier.getAsDouble()));
  }

  public Command controllerDriveFieldCentricSnapCommand() {
    return startRun(() -> {
      double snapDir = 90.0;
      double angle = getPose().getRotation().getDegrees();
      angle = Math.round(angle / snapDir) * snapDir;
      snapDirection = Rotation2d.fromDegrees(angle);
    }, () -> {
      controllerDriveFieldCentricFacingDir(snapDirection);
    });
  }

  public void controllerDriveFieldCentricFlickStick() {
    double x = Constants.controller.getRightX();
    double y = Constants.controller.getRightY();

    double dist = Math.sqrt(x * x + y * y);
    double angle = Math.atan2(x, y);

    dist = MathUtil.clamp(dist, 0.0, 1.0);
    if (dist < 0.05)
      dist = 0;

    isUsingFlickStick = true;
    controllerDriveFieldCentricFacingDir(
        Rotation2d.fromRadians(angle).rotateBy(Rotation2d.k180deg),
        dist);

  }

  public Command controllerDriveFieldCentricFlickStickCommand() {
    return runEnd(this::controllerDriveFieldCentricFlickStick, () -> isUsingFlickStick = false);
  }

  public void controllerDriveRobotCentricFlickStick() {
    double x = Constants.controller.getRightX();
    double y = Constants.controller.getRightY();

    double dist = Math.sqrt(x * x + y * y);
    double angle = Math.atan2(x, y);

    dist = MathUtil.clamp(dist, 0.0, 1.0);
    if (dist < 0.05)
      dist = 0;

    isUsingFlickStick = true;
    controllerDriveRobotCentricFacingDir(
        Rotation2d.fromRadians(angle).rotateBy(Rotation2d.k180deg),
        dist);

  }

  public Command controllerDriveRobotCentricFlickStickCommand() {
    return runEnd(this::controllerDriveRobotCentricFlickStick, () -> isUsingFlickStick = false);
  }

  // public void driveRobotCentric(DoubleSupplier xSpeed, DoubleSupplier ySpeed,
  // DoubleSupplier zRotation) {
  // drive.driveCartesian(xSpeed.getAsDouble(), ySpeed.getAsDouble(),
  // zRotation.getAsDouble());
  // }

  public Command driveRobotCentricCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation) {
    return run(
        () -> driveRobotCentric(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  public void controllerDriveRobotCentric() {
    double driveX = Constants.controller.getDriveX();
    double driveY = -Constants.controller.getDriveY();
    double driveTurn = -Constants.controller.getDriveTurn();

    driveX = Math.signum(driveX) * driveX * driveX;
    driveY = Math.signum(driveY) * driveY * driveY;
    driveTurn = Math.signum(driveTurn) * driveTurn * driveTurn;

    double speed = getMaxSpeed();

    driveX *= speed;
    driveY *= speed;
    driveTurn *= speed;

    driveRobotCentric(driveX, driveY, driveTurn);
  }

  public Command controllerDriveRobotCentricCommand = run(this::controllerDriveRobotCentric)
      .withName("driveControllerRobotCentric");

  /**
   * Updates current pose using encoder positions
   */
  public void updatePose() {
    poseEstimator.update(Rotation2d.fromDegrees(-gyro.getYaw()), new MecanumDriveWheelPositions(
        Units.Meters.of(flEncoder.getDistance()),
        Units.Meters.of(frEncoder.getDistance()),
        Units.Meters.of(blEncoder.getDistance()),
        Units.Meters.of(brEncoder.getDistance())));
    field.setRobotPose(getPose());
    if (limelightPoseSupplier != null && limelightTimestampSupplier != null && turretAngle != null && doUpdate != null
        && doUpdate.get()) {
      Pose2d robotPose;

      field.getObject("limelight").setPose(limelightPoseSupplier.get());

      double visionRotStdev = 9999999;
      if (limelightMt2Supplier != null && limelightMt2Supplier.getAsBoolean() == false) {
        visionRotStdev = 1.0;
        robotPose = cameraToBotRaw(limelightPoseSupplier.get());
      } else {
        robotPose = cameraToBot(limelightPoseSupplier.get());
      }
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, visionRotStdev));
      poseEstimator.addVisionMeasurement(robotPose, limelightTimestampSupplier.getAsDouble());
    } else {
      field.getObject("limelight").setPose(getEstimatedCameraPose());
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPos(Pose2d pose) {
    poseEstimator.resetPose(pose);
    poseEstimator.resetPose(pose);

  }

  public void resetPos() {
    Pose2d pose = new Pose2d(Kinematics.HUB_POSITION_2D.plus(new Translation2d(-1, 0)), Rotation2d.kZero);
    resetPos(pose);
    // gyro.setAngleAdjustment(
    // gyro.getAngleAdjustment()
    // + pose.getRotation().getDegrees() - gyro.getAngle());

    currentVelocity = new ChassisSpeeds();
  }

  public void voltageDrive() {
    flMotor.setVoltage(-volts.getDouble(0));
    frMotor.setVoltage(-volts.getDouble(0));
    blMotor.setVoltage(-volts.getDouble(0));
    brMotor.setVoltage(-volts.getDouble(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    updateVelocity();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stop() {
    drive.driveCartesian(0, 0, 0);
  }

  /*
   * Predicts future robot pose based on current velocities
   * 
   * @param(dt) time in seconds to predict into the future, higher values are more
   * unreliable as there is no smoothing or acceleration accounted for
   * Note: this is a simple extrapolation and does not account for acceleration or
   * changes in velocity
   */

  ChassisSpeeds currentVelocity = new ChassisSpeeds();

  private void updateVelocity() {
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(
        flEncoder.getRate(),
        frEncoder.getRate(),
        blEncoder.getRate(),
        brEncoder.getRate()));

    // double inter = 0;

    currentVelocity = speeds;

    // currentVelocity = new Transform2d(
    // MathUtil.interpolate(speeds.vxMetersPerSecond, currentVelocity.getX(),
    // inter),
    // MathUtil.interpolate(speeds.vyMetersPerSecond, currentVelocity.getY(),
    // inter),
    // Rotation2d.fromRadians(
    // MathUtil.interpolate(speeds.omegaRadiansPerSecond,
    // currentVelocity.getRotation().getRadians(), inter)));
  }

  public ChassisSpeeds getVelocity() {
    return currentVelocity;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(currentVelocity, getPose().getRotation());
  }

  public Pose2d getFutureRobotPose2d(double dt) {
    Pose2d pose = getPose();
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(
        flEncoder.getRate(),
        frEncoder.getRate(),
        blEncoder.getRate(),
        brEncoder.getRate()));

    Twist2d delta = speeds.toTwist2d(dt);

    return pose.plus(new Transform2d(delta.dx, delta.dy, Rotation2d.fromRadians(delta.dtheta)));
  }

  public Pose2d getSmoothFutureRobotPose2d(double dt) {
    Twist2d delta = getVelocity().toTwist2d(dt);
    Transform2d trans = new Transform2d(delta.dx, delta.dy, Rotation2d.fromRadians(delta.dtheta));
    return getPose().transformBy(trans);
  }

  /*
   * Value of 0.0004s accounts for the delay with the turret, any more is
   * erraneous
   */
  public Pose2d getFutureRobotPose2d() {
    return getFutureRobotPose2d(0.0004);
  }

  /* Gets the pose of the turret with the bots position */
  public Pose2d botToTurret(Pose2d botPose) {
    return botPose.transformBy(
        new Transform2d(-Constants.CENTER_OF_BOT_TO_CENTER_OF_TURRET.in(Units.Meters), 0,
            Rotation2d.k180deg));
  }

  /* Transforms a bot pose to a camera pose */
  public Pose2d botToCamera(Pose2d botPose) {
    return botPose.transformBy(
        new Transform2d(-Constants.CENTER_OF_BOT_TO_CENTER_OF_TURRET.in(Units.Meters), 0,
            Rotation2d.fromDegrees(180 + turretSubsystem.getHeading())))
        .transformBy(new Transform2d(Constants.TURRET_RADIUS.in(Units.Meters), 0, Rotation2d.kZero));
  }

  /* Gets the pose of the turret with the bots position */
  public Pose2d getTurretPose() {
    return botToTurret(getPose());
  }

  /* Gets the pose of the camera with the bots position and turret angle */
  public Pose2d getEstimatedCameraPose() {
    return botToCamera(getPose());
  }

  /* Extrapolates the instantaneous speed of the turret in m/s */
  public Translation2d getTurretSpeed() {
    Pose2d currentTurretPose = getTurretPose();
    Pose2d futureTurretPose = botToTurret(getSmoothFutureRobotPose2d(0.05));
    Translation2d deltaTranslation = futureTurretPose.getTranslation().minus(currentTurretPose.getTranslation());
    return deltaTranslation.div(0.05);
  }

  /*
   * Get the predicted hub location to aim at such that a ball with a given flight
   * time will reach the hub
   */
  public Translation2d getGhostHubTranslation2d(double flightTime) {
    Translation2d turretSpeed = getTurretSpeed();
    Translation2d hubTranslation2d = Kinematics.HUB_POSITION_2D.minus(turretSpeed.times(flightTime));
    return hubTranslation2d;
  }

  /* Get predicted hub transform2d for a given flight time */
  public Transform2d getPredictedHubTransform2d(double flightTime) {
    return Kinematics.getHubTransform2d(botToTurret(getFutureRobotPose2d()), getGhostHubTranslation2d(flightTime));
  }

  public Pose2d cameraToBotRaw(Pose2d cameraPose) {
    return cameraPose
        .transformBy(new Transform2d(
            -Constants.TURRET_RADIUS.in(Units.Meters),
            0,
            Rotation2d.fromDegrees(-turretSubsystem.getHeading())))
        .transformBy(new Transform2d(
            -Constants.CENTER_OF_BOT_TO_CENTER_OF_TURRET.in(Units.Meters),
            0,
            Rotation2d.k180deg));
  }

  public Pose2d cameraToBot(Pose2d cameraPose) {
    // Not using MT1 rotation

    Rotation2d robotRot = getPose().getRotation();

    return new Pose2d(
        cameraPose.getTranslation().plus(
            new Translation2d(Constants.TURRET_RADIUS.in(Units.Meters), 0)
                .rotateBy(Rotation2d.fromDegrees(robotRot.getDegrees() + (turretSubsystem.getHeading()))))
            .plus(new Translation2d(Constants.CENTER_OF_BOT_TO_CENTER_OF_TURRET.in(Units.Meters),
                robotRot)),
        robotRot);

    // cameraPose.transformBy(new Transform2d(-Constants.TURRET_RADIUS.in(Meters),
    // 0, robotAngle.plus(new Rotation2d(turretAngle))));
    // return Pose2d.kZero;
  }

  // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public double remainingHubSwitchTime() {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime > 130) {
      // Transition shift, hub is active.
      return matchTime - 130;
    } else if (matchTime > 105) {
      // Shift 1
      return matchTime - 105;
    } else if (matchTime > 80) {
      // Shift 2
      return matchTime - 80;
    } else if (matchTime > 55) {
      // Shift 3
      return matchTime - 55;
    } else if (matchTime > 30) {
      // Shift 4
      return matchTime - 30;
    } else {
      // End game, hub always active.
      return matchTime;
    }
  }
}
