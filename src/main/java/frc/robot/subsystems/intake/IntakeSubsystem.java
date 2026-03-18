package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

    @AutoLogOutput(key = kLogPath + "/State")
    @Getter
    private IntakeState m_state = IntakeState.STOWED;

    public IntakeSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new IntakeIOTalonFX(kRollersId, kPivotId, Constants.kCANBusRio);
            case SIM -> new IntakeIOSim();
            default -> new IntakeIO() {};
        };
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
                Logger.recordOutput("DeployedPosRot", kPivotDeployedPosition.getRotations());
                Logger.recordOutput("DeployedCurrentPositionRot", m_inputs.pivotPositionRot);
                Logger.recordOutput("DeployedTolerance", kPivotDeployedTolerance.getRotations());
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
                m_io.runRollersVoltage(kRollersIntakeVoltage);
            }
            case STOWED -> {
                m_io.runPivotPosition(kPivotStowedPosition);
                m_io.runRollersVoltage(0.0);
            }
            case NONE -> {
            }
        }
    }

    public enum IntakeState {
        DEPLOYED,
        STOWED,
        NONE;
    }
}
