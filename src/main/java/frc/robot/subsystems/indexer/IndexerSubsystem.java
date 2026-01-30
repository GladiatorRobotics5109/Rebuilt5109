package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.IndexerConstants.*;

public class IndexerSubsystem extends SubsystemBase {
    private final IndexerIO m_io;
    private final IndexerIOInputsAutoLogged m_inputs;

    public IndexerSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new IndexerIOTalonFX();
            case SIM -> new IndexerIOSim();
            case REPLAY -> new IndexerIO() {};
        };
        m_inputs = new IndexerIOInputsAutoLogged();
    }

    public void runVoltage(double volts) {
        m_io.runVoltage(volts);
    }

    public void stop() {
        runVoltage(0.0);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }
    }
}
