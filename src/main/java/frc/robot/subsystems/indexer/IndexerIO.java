package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public boolean indexerConnected = false;
        public double indexerPositionRad = 0.0;
        public double indexerVelocityRadPerSec = 0.0;
        public double indexerAppliedVolts = 0.0;
        public double indexerStatorCurrentAmps = 0.0;
        public double indexerSupplyCurrentAmps = 0.0;
        public double indexerTempCelsius = 0.0;

        public double kickupPositionRad = 0.0;
        public double kickupVelocityRadPerSec = 0.0;
        public double kickupAppliedVolts = 0.0;
        public double kickupStatorCurrentAmps = 0.0;
        public double kickupSupplyCurrentAmps = 0.0;
        public double kickupTempCelsius = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void runVoltage(double indexerVolts, double kickupVolts) {}
}
