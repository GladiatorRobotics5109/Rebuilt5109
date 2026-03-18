package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableFF;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoggedTunablePID;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.Constants.TurretConstants.*;

public class TurretSubsystem extends SubsystemBase {
    private final TurretIO m_io;
    private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();

    private final LoggedTunablePID m_pid = new LoggedTunablePID(kLogPath + "/PID", kP, kI, kD);
    private final LoggedTunableFF m_ff = new LoggedTunableFF(kLogPath + "/FF", kS, kV, kA, 0.0);
    private final LoggedTunableNumber m_mmAccel = new LoggedTunableNumber(
        kLogPath + "/MotionMagicAccel",
        kMotionMagicCruiseAccelerationRadPerSecSq
    );
    private final LoggedTunableNumber m_mmCruiseVel = new LoggedTunableNumber(
        kLogPath + "/MotionMagicCruiseVelocity",
        kMotionMagicCruiseVelocityRadPerSec
    );

    private Supplier<Rotation2d> m_desiredPosition;
    @AutoLogOutput(key = kLogPath + "/HasDesiredPosition")
    private boolean m_hasDesiredPosition;

    public TurretSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new TurretIOTalonFX(kId, Constants.kCANBusRio);
            case SIM -> new TurretIOSim();
            default -> new TurretIO() {};
        };
    }

    public void runPosition(Rotation2d position) {
        runPosition(() -> position);
    }

    public void runPosition(Supplier<Rotation2d> position) {
        m_desiredPosition = position;
        m_hasDesiredPosition = true;
    }

    public void runVoltage(double volts) {
        m_hasDesiredPosition = false;
        m_io.runVoltage(volts);
    }

    public void stop() {
        runVoltage(0.0);
    }

    public void setPosition(Rotation2d position) {
        m_io.setPosition(position.getRadians());
    }

    public Rotation2d getPosition() { return Rotation2d.fromRadians(m_inputs.positionRad); }

    public double getVelocityRadPerSec() { return m_inputs.velocityRadPerSec; }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }
        else if (m_hasDesiredPosition) {
            Rotation2d desired = m_desiredPosition.get();
            Logger.recordOutput(kLogPath + "/DesiredPosiiton", desired);
            m_io.runPosition(desired.getRadians());
        }

        if (m_pid.hasChanged(hashCode())) {
            m_io.setPID(m_pid.getP(), m_pid.getI(), m_pid.getD());
        }

        if (m_ff.hasChanged(hashCode())) {
            m_io.setFF(m_ff.getS(), m_ff.getV(), m_ff.getA());
        }

        if (m_mmAccel.hasChanged(hashCode()) || m_mmCruiseVel.hasChanged(hashCode())) {
            m_io.setMotionMagic(m_mmCruiseVel.get(), m_mmAccel.get());
        }

        RobotState.getInstance().updateTurret(getPosition(), m_inputs.velocityRadPerSec);
    }
}
