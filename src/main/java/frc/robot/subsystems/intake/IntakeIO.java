package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public boolean pivotConnected = false;
        public double pivotPositionRot = 0.0;
        public double pivotVelocityRPM = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotStatorCurrentAmps = 0.0;
        public double pivotSupplyCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public boolean rollersConnected = false;
        public double rollersPositionRot = 0.0;
        public double rollersVelocityRPM = 0.0;
        public double rollersAppliedVolts = 0.0;
        public double rollersStatorCurrentAmps = 0.0;
        public double rollersSupplyCurrentAmps = 0.0;
        public double rollersTempCelsius = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void runPivotVoltage(double volts) {}

    default void runPivotPosition(Rotation2d position) {}

    default void runRollersVoltage(double volts) {}
}
