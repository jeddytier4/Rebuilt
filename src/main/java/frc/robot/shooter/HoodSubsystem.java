// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase { 
    private Servo primaryHoodServo;
    private Servo secondaryHoodServo;
    public GenericEntry hoodSetpoint;

    private double minVal = 0.2;
    private double maxVal = 0.68;

    /** Creates a new ExampleSubsystem. */
    public HoodSubsystem() {
        primaryHoodServo = new Servo(8);
        secondaryHoodServo = new Servo(9);
        Shuffleboard.getTab("hood").add(primaryHoodServo);
        hoodSetpoint = Shuffleboard.getTab("hood").add("hood-value-setter", 0).getEntry();
        Shuffleboard.getTab("hood").addDouble("controlelr", ()->Constants.controller.getHoodDisplacement());
    }

    /**
     * Set hood position directly
     * 
     * @param position Position between 0.0 and 1.0
     */
    public void setPos(double position) {
        position = MathUtil.clamp(position * (maxVal - minVal) + minVal, minVal, maxVal);
        primaryHoodServo.set(position);
        secondaryHoodServo.set(position);
    }

    public double getPos() {
        return (primaryHoodServo.get() - minVal) / (maxVal - minVal);
    }

    /**
     * Set hood position based on distance to target
     * 
     * @param distance Distance to target. Must be between 0 and 100 mm.
     */
    public void setDistance(Distance distance) {
        double position = Math.min(Math.max(distance.in(Units.Millimeters) / 100.0, 0.0), 1.0);
        setPos(position);
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
