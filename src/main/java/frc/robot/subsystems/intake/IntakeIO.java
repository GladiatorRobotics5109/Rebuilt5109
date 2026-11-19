package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void runPivotVoltage(double volts) {}

    default void runPivotPosition(Rotation2d position) {}

    default void runRollersVoltage(double volts) {}
}
