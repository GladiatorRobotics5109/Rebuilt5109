package frc.robot.subsystems.hood;

import static frc.robot.Constants.IndexerConstants.kId;
import static frc.robot.Constants.IndexerConstants.kLogPath;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.LoggedTunableFF;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoggedTunablePID;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import lombok.Getter;
import lombok.experimental.Accessors;

import static frc.robot.Constants.HoodConstants.*;

public class HoodSubsystem extends SubsystemBase {
    private final HoodIO m_io;
    private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();

    private DoubleSupplier m_desiredAngleRad;

    private final LoggedTunablePID m_pid = new LoggedTunablePID(kLogPath + "/PID", kP, kI, kD);
    private final LoggedTunableFF m_ff = new LoggedTunableFF(kLogPath + "/FF", kS, kV, kA, kG);
    private final LoggedTunableNumber m_mmAccel = new LoggedTunableNumber(
        TurretConstants.kLogPath + "/MotionMagicAccelRadPerSecSq",
        kMotionMagicCruiseAccelerationRadPerSecSq
    );
    private final LoggedTunableNumber m_mmCruiseVel = new LoggedTunableNumber(
        TurretConstants.kLogPath + "/MotionMagicCruiseVelocityRadPerSec",
        kMotionMagicCruiseVelocityRadPerSec
    );

    @Accessors(fluent = true)
    @Getter
    @AutoLogOutput(key = kLogPath + "/HasDesiredAngle")
    public boolean m_hasDesiredAngle;

    public HoodSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new HoodIOTalonFX(kId, Constants.kCANBusRio);
            case SIM -> new HoodIOSim();
            default -> new HoodIO() {};
        };
    }

    public void runAngle(double angleRad) {
        runAngle(() -> angleRad);
    }

    public void runAngle(DoubleSupplier angleRad) {
        m_desiredAngleRad = angleRad;
        m_hasDesiredAngle = true;
    }

    public void runVoltage(double volts) {
        m_hasDesiredAngle = false;
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
        else if (m_hasDesiredAngle) {
            double desired = MathUtil.clamp(
                m_desiredAngleRad.getAsDouble(),
                kMinAngle.getRadians(),
                kMaxAngle.getRadians()
            );

            m_io.runPosition(desired);

            Logger.recordOutput(kLogPath + "/DesiredAngle", desired);
        }

        if (m_pid.hasChanged(hashCode())) {
            m_io.setPID(m_pid.getP(), m_pid.getI(), m_pid.getD());
        }

        if (m_ff.hasChanged(hashCode())) {
            m_io.setFF(m_ff.getS(), m_ff.getV(), m_ff.getA(), m_ff.getG());
        }

        if (m_mmAccel.hasChanged(hashCode()) || m_mmCruiseVel.hasChanged(hashCode())) {
            m_io.setMotionMagic(m_mmCruiseVel.get(), m_mmAccel.get());
        }

        RobotState.getInstance().updateHood(Rotation2d.fromRadians(m_inputs.positionRad));
    }
}
