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

    default void setPosition(double positionRad) {}
}
