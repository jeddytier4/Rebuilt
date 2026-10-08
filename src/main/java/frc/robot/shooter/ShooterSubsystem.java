// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public TalonFX primaryShooter;
    public TalonFX secondaryShooter;

    public VelocityVoltage velocityRequest;
    public VoltageOut voltageRequest;
    public DutyCycleOut dutyCycleRequest;
    
    public SysIdRoutine sysId;
    
    public GenericEntry targetRPM;
    public GenericEntry RPMMod;

    public Orchestra orchestra;

    /** Creates a new ExampleSubsystem. */
    public ShooterSubsystem() {
        primaryShooter = new TalonFX(Constants.PRIMARY_SHOOTER);
        secondaryShooter = new TalonFX(Constants.SECONDARY_SHOOTER);

        Shuffleboard.getTab("pid").addDouble("rpm",
                () -> Units.RotationsPerSecond.of(primaryShooter.getVelocity().getValueAsDouble()).in(Units.RPM));
        Shuffleboard.getTab("pid").addDouble("rpm-sec",
                () -> Units.RotationsPerSecond.of(secondaryShooter.getVelocity().getValueAsDouble()).in(Units.RPM));
        Shuffleboard.getTab("pid").addDouble("motor_curr_target", () -> Units.RotationsPerSecond
                .of(primaryShooter.getClosedLoopReference().getValueAsDouble()).in(Units.RPM));

        RPMMod = Shuffleboard.getTab("pid").add("RPM-MODIFIER", 1).getEntry();

        targetRPM = Shuffleboard.getTab("pid").add("RPM-TARGET", 3000).getEntry();

        secondaryShooter.setControl(new Follower(Constants.PRIMARY_SHOOTER, MotorAlignmentValue.Opposed));

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.SHOOTER_kP;
        slot0Configs.kI = Constants.SHOOTER_kI;
        slot0Configs.kD = Constants.SHOOTER_kD;

        slot0Configs.kS = Constants.SHOOTER_kS;
        slot0Configs.kV = Constants.SHOOTER_kV;

        primaryShooter.getConfigurator().apply(slot0Configs);

        velocityRequest = new VelocityVoltage(0).withSlot(0);
        voltageRequest = new VoltageOut(0);
        dutyCycleRequest = new DutyCycleOut(0);

        orchestra = new Orchestra();

        orchestra.addInstrument(secondaryShooter);

        orchestra.loadMusic("mc.chrp");

        this.sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
                voltage -> {
                    primaryShooter.setControl(voltageRequest.withOutput(voltage));
                },
                log -> {
                    log.motor("shooter")
                            .voltage(Units.Volts.of(primaryShooter.getMotorVoltage().getValueAsDouble()))
                            .angularPosition(Units.Rotations.of(primaryShooter.getPosition().getValueAsDouble()))
                            .angularVelocity(
                                    Units.RotationsPerSecond.of(primaryShooter.getVelocity().getValueAsDouble()));
                },
                this));
    }

    public void setVelocity(AngularVelocity speed) {
        speed = speed.times(RPMMod.getDouble(1));
        primaryShooter.setControl(velocityRequest.withVelocity(speed).withFeedForward(Constants.SHOOTER_kF));

    }

    public void setDutyCycle(double speed) {
        speed = speed * RPMMod.getDouble(1);
        primaryShooter.setControl(dutyCycleRequest.withOutput(speed));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
