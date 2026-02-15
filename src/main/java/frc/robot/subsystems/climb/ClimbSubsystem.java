package frc.robot.subsystems.climb;

import frc.robot.Constants;

public class ClimbSubsystem {
    private final ClimbIO m_io;
    private final ClimbIOInputsAutoLogged m_inputs = new ClimbIOInputsAutoLogged();

    public ClimbSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new ClimbIOTalonFX();
            case SIM -> new ClimbIOSim();
            default -> new ClimbIO() {};
        };
    }
}
