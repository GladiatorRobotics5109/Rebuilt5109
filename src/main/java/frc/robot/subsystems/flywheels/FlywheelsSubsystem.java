package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.FlywheelsConstants.*;

public class FlywheelsSubsystem extends SubsystemBase {
    // private final LoggedTunablePID m_pid = new LoggedTunablePID(kLogPath + "/PID", kP, kI, kD);

    private final FlywheelsIO m_io;
    private final FlywheelsIOInputsAutoLogged m_inputs = new FlywheelsIOInputsAutoLogged();

    private DoubleSupplier m_desiredVelocity;

    @Accessors(fluent = true)
    @Getter
    @AutoLogOutput(key = kLogPath + "/HasDesiredVelocity")
    public boolean m_hasDesiredVelocity;

    public FlywheelsSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            // case REAL -> new FlywheelsIOTalonFX(kId, Constants.kCANBusRio);
            case SIM -> new FlywheelsIOSim();
            default -> new FlywheelsIO() {};
        };
    }

    public void runVelocity(DoubleSupplier velocityRPM) {
        m_desiredVelocity = velocityRPM;
        m_hasDesiredVelocity = true;
    }

    public void runVelocity(double velocityRPM) {
        runVelocity(() -> velocityRPM);
    }

    public void runVoltage(double volts) {
        m_hasDesiredVelocity = false;
        m_desiredVelocity = () -> 0.0;

        m_io.setVoltage(volts);
    }

    public void stop() {
        runVoltage(0.0);
    }

    /**
     * Simulates what shooting a ball is like. Should not be called on the real robot.
     */
    public void simulateShot() {
        if (Constants.kCurrentMode == Mode.SIM && m_io instanceof FlywheelsIOSim sim) {
            sim.simulateShot();
        }
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            m_io.setVelocity(0.0);
        }

        // if (m_pid.hasChanged(hashCode())) {
        //     m_io.setPID(m_pid.getP(), m_pid.getI(), m_pid.getD());
        // }

        if (m_hasDesiredVelocity) {
            double desired = m_desiredVelocity.getAsDouble();
            Logger.recordOutput(kLogPath + "/DesiredVelocityRPM", desired);
            m_io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(desired));
        }

        RobotState.getInstance().updateFlywheels(
            Units.radiansPerSecondToRotationsPerMinute(m_inputs.velocityRadPerSec)
        );
    }
}
