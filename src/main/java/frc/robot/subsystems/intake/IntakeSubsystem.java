package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

    @AutoLogOutput(key = kLogPath + "/State")
    @Getter
    private IntakeState m_state = IntakeState.STOWED;

    @AutoLogOutput(key = kLogPath + "/Reverse")
    @Getter
    @Setter
    private boolean m_reverse;

    public IntakeSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new IntakeIOTalonFX(kRollersId, kPivotId, Constants.kCANBusRio);
            case SIM -> new IntakeIOSim();
            default -> new IntakeIO() {};
        };

        Logger.recordOutput(kLogPath + "/PivotDesiredPosition", Rotation2d.kZero);
    }

    public void deploy() {
        m_state = IntakeState.DEPLOYED;
    }

    public void stow() {
        m_state = IntakeState.STOWED;
    }

    public void stop() {
        m_state = IntakeState.NONE;
    }

    public void runPivotVoltage(double volts) {
        m_io.runPivotVoltage(volts);
        m_state = IntakeState.NONE;
    }

    public void runRollersVoltage(double volts) {
        m_io.runRollersVoltage(volts);
        m_state = IntakeState.NONE;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }

        switch (m_state) {
            case DEPLOYED -> {
                Logger.recordOutput(kLogPath + "/PivotDesiredPosition", kPivotDeployedPosition);
                if (MathUtil.isNear(
                    kPivotDeployedPosition.getRotations(),
                    m_inputs.pivotPositionRot,
                    kPivotDeployedTolerance.getRotations()
                )) {
                    m_io.runPivotVoltage(kPviotDeployedHoldingVoltage);
                }
                else {
                    m_io.runPivotPosition(kPivotDeployedPosition);
                }
                m_io.runRollersVoltage(m_reverse ? kRollersReverseVoltage : kRollersIntakeVoltage);
            }
            case STOWED -> {
                Logger.recordOutput(kLogPath + "/PivotDesiredPosition", kPivotStowedPosition);
                m_io.runPivotPosition(kPivotStowedPosition);
                m_io.runRollersVoltage(0.0);
            }
            case NONE -> {
                Logger.recordOutput(kLogPath + "/PivotDesiredPosition", 0.0);
            }
        }
    }

    public enum IntakeState {
        DEPLOYED,
        STOWED,
        NONE;
    }
}
