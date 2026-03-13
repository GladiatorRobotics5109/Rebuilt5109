package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.util.TolerancedBangBang;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.FlywheelsConstants.*;

public class FlywheelsSubsystem extends SubsystemBase {
    private final FlywheelsIO m_io;
    private final FlywheelsIOInputsAutoLogged m_inputs = new FlywheelsIOInputsAutoLogged();

    private final TolerancedBangBang m_bang = new TolerancedBangBang(kBangBangTolerance);
    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(kS, kV, kA);

    private DoubleSupplier m_desiredVelocity;
    @Accessors(fluent = true)
    @Getter
    private boolean m_hasDesiredVelocity;

    public FlywheelsSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new FlywheelsIOTalonFX(kId, Constants.kCANBusRio);
            case SIM -> new FlywheelsIOSim();
            default -> new FlywheelsIO() {};
        };
    }

    public void runVelocity(DoubleSupplier velocityRPM) {
        m_desiredVelocity = velocityRPM;
        m_hasDesiredVelocity = true;
    }

    public void runVelocity(double velocityRPM) { runVelocity(() -> velocityRPM); }

    public void runVoltage(double volts) {
        m_hasDesiredVelocity = false;
        m_desiredVelocity = () -> 0.0;

        m_io.setVoltage(volts);
    }

    public void stop() { runVoltage(0.0); }

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
            stop();
        }

        double desired = m_desiredVelocity.getAsDouble();
        if (m_hasDesiredVelocity) {
            double bang = 12 * m_bang.calculate(m_inputs.velocityRPM, desired);
            double ff = m_ff.calculate(desired);
            // Logger.recordOutput(kLogPath + "/control_desired", desired);
            // Logger.recordOutput(kLogPath + "/control_velocity", m_inputs.velocityRPM);
            Logger.recordOutput(kLogPath + "/BangOutput", bang);
            Logger.recordOutput(kLogPath + "/FFOutput", ff);
            m_io.setVoltage(bang + ff);
        }

        RobotState.getInstance().updateFlywheels(
            m_inputs.velocityRPM,
            desired,
            m_hasDesiredVelocity
        );
    }
}
