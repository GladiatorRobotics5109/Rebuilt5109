package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.Constants.TurretConstants.*;

public class TurretSubsystem extends SubsystemBase {
    private final TurretIO m_io;
    private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();

    private Supplier<Rotation2d> m_desiredPosition;
    @AutoLogOutput(key = kLogPath + "/HasDesiredPosition")
    private boolean m_hasDesiredPosition;

    public TurretSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            //case REAL -> new TurretIOTalonFX(kId, Constants.kCANBusCANivore);
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
        m_io.setVoltage(volts);
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
        else if (m_hasDesiredPosition) {
            Rotation2d desired = m_desiredPosition.get();
            Logger.recordOutput(kLogPath + "/DesiredPosiiton", desired);
            m_io.setPosition(desired.getRadians());
        }

        RobotState.getInstance().updateTurret(Rotation2d.fromRadians(m_inputs.positionRad));
    }
}
