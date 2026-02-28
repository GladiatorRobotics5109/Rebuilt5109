package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(TurretIO.TurretIOInputs inputs) {}

    default void setPosition(double positionRad) {}

    default void setVoltage(double volts) {}

    default void setPID(double p, double i, double d) {}

    default void setFF(double s, double v, double a) {}

    default void setMotionMagic(double cruiseVelocityRadPerSec, double accelerationRadPerSecSq) {}
}
