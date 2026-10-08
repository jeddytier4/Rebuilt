// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    public SparkFlex climbMotor;
    public RelativeEncoder climbEncoder;
    // public ColorSensorV3 colorSensor;
    // private WrappingDutyCycleEncoder temp;
    // public DigitalInput limitSwitch;


    private final double maxExtent = 6.5 + 0.8;

    /** Creates a new ExampleSubsystem. */
    public ClimbSubsystem() {
        // TODO: config in rev hardware client
        climbMotor = new SparkFlex(Constants.CLIMB_MOTOR, MotorType.kBrushless);

        double cP = 0.016;
        double cI = 0.00;
        double cD = 0.005;


        climbEncoder = climbMotor.getEncoder();

        climbMotor.configure(
                new SparkFlexConfig()
                        .apply(new EncoderConfig().positionConversionFactor(Constants.CLIMB_POSISION_CONVERSION_FACTOR)
                                .velocityConversionFactor(Constants.CLIMB_VELOCITY_CONVERSION_FACTOR))
                        .apply(new SparkMaxConfig().idleMode(IdleMode.kBrake))
                        .apply(new ClosedLoopConfig()
                                .pid(cP, cI, cD)
                                .maxOutput(0.5)
                                .minOutput(-0.5))
                .apply(new SoftLimitConfig()
                .forwardSoftLimit(maxExtent).forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0.0).reverseSoftLimitEnabled(true))
                ,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shuffleboard.getTab("climb").add("climbMotor", climbMotor);
        Shuffleboard.getTab("climb").addDouble("climbCurrent", climbMotor::getOutputCurrent);
        Shuffleboard.getTab("climb").addDouble("climbPos", climbEncoder::getPosition);
        Shuffleboard.getTab("climb").addDouble("climbVel", climbEncoder::getVelocity);

        Shuffleboard.getTab("climb").add("Reset climb 0",
                runOnce(() -> resetPosition(0.0)).ignoringDisable(true));
        Shuffleboard.getTab("climb").add("Reset climb max (" + maxExtent + ")",
                runOnce(() -> resetPosition(maxExtent)).ignoringDisable(true));

    }

    public double getPos() {
        return climbEncoder.getPosition();
    }

    public void setTarget(double pos) throws Exception {
        throw new Exception("Adjust PID and max speed in REV first!!!!!");
        // climbMotor.getClosedLoopController().setSetpoint(pos, ControlType.kPosition);
    }

    public void resetPosition(double position) {
        climbEncoder.setPosition(position);
    };

    public void setPower(double power) {
        climbMotor.set(power);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public Command setPowerCommand(double power) {
        return run(() -> setPower(power)).withName("Climb power (" + power + ")");
    }

    public Command climbDefaultCommand() {
        return setPowerCommand(0);
    }

    public boolean isHaulted() {
        return climbMotor.getOutputCurrent() > 1 && climbEncoder.getVelocity() < 1;
    }

    public Command homeClimbCommand() {
        return run(() -> setPower(-0.1)).until(() -> true);
    }

    double setpointDefaultSpeed = 0.5;
    double setpointDir = 1;

    public Command runToSetpointCommand(double setpoint) {

        return startRun(  () -> setpointDir = (setpoint - getPos() >= 0 ? 1 : -1), 
                        () -> setPower(setpointDir * setpointDefaultSpeed))
                        .until(() -> (setpointDir > 0 && (setpoint - getPos()) <= 0) || (setpointDir < 0 && (getPos() - setpoint) <= 0));
    }
}
