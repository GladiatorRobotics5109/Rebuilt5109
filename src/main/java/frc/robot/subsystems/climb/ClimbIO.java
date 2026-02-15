package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {

    }

    default void updateInputs(ClimbIOInputs inputs) {}
}
