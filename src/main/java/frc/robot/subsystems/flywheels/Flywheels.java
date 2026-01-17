package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunablePID;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.FlywheelsConstants.*;

public class Flywheels extends SubsystemBase {
    private final LoggedTunablePID m_pid = new LoggedTunablePID(kLogPath + "/PID", kP, kI, kD);

    private final FlywheelsIO m_io;
    private final FlywheelsIOInputsAutoLogged m_inputs = new FlywheelsIOInputsAutoLogged();

    @Getter
    @AutoLogOutput(key = kLogPath + "/DesiredVelocityRPM")
    private double m_desiredVelocityRPM;

    @Accessors(fluent = true)
    @Getter
    @AutoLogOutput(key = kLogPath + "/HasDesiredVelocity")
    public boolean m_hasDesiredVelocity;

    public Flywheels(FlywheelsIO io) {
        m_io = io;
    }

    public void runVelocity(double velocityRPM) {
        m_io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
        m_hasDesiredVelocity = true;
        m_desiredVelocityRPM = velocityRPM;
    }

    public void runVoltage(double volts) {
        m_io.setVoltage(volts);
        m_desiredVelocityRPM = 0.0;
        m_hasDesiredVelocity = false;
    }

    public void stop() {
        runVoltage(0.0);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Flywheels", m_inputs);

        if (DriverStation.isDisabled()) {
            m_io.setVelocity(0.0);
        }

        if (m_pid.hasChanged(hashCode())) {
            m_io.setPID(m_pid.getP(), m_pid.getI(), m_pid.getD());
        }
    }
}
