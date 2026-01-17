package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
    @AutoLog
    class FlywheelsIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(FlywheelsIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setVelocity(double velocityRadPerSec) {}

    default void setPID(double p, double i, double d) {}
}
