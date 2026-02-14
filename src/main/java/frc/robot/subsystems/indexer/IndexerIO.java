package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void runVoltage(double volts) {}
}
