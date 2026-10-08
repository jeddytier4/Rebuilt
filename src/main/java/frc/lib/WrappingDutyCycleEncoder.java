package frc.lib;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class WrappingDutyCycleEncoder extends DutyCycleEncoder {
    private double offset = 0;
    private DoubleSupplier motorSupplier;
    private double prevReading = 0;
    private double inverted = 1;

    public WrappingDutyCycleEncoder(int port, DoubleSupplier motorVelocity, double offset, boolean inverted) {
        super(port);

        this.offset = offset;
        this.motorSupplier = motorVelocity;
        this.prevReading = 0;
        this.inverted = inverted ? -1 : 1;
    }

    @Override
    public double get() {
        return inverted * (super.get() + offset);

    }

    public void update() {
        double currReading = Math.round(super.get() * 10) / 10.0;
        double velocity = Math.round(motorSupplier.getAsDouble());

        if (velocity * inverted < 0 && currReading > prevReading) {
            offset -= 1;
        } else if (velocity * inverted > 0 && currReading < prevReading) {
            offset += 1;
        }

        prevReading = currReading;
    }

}
