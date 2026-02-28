package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void runVoltage(double volts) {}

    default void runPosition(double positionRad) {}

    default void setPID(double p, double i, double d) {}

    default void setFF(double s, double v, double a, double g) {}

    default void setMotionMagic(double cruiseVelocityRadPerSec, double accelerationRadPerSecSq) {}
}
