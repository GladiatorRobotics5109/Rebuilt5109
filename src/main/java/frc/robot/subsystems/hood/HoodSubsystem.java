package frc.robot.subsystems.hood;

import static frc.robot.Constants.IndexerConstants.kId;
import static frc.robot.Constants.IndexerConstants.kLogPath;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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

    public void runAngle(double angleDeg) {
        m_desiredAngleRad = () -> Math.toRadians(angleDeg);
        m_hasDesiredAngle = true;
    }

    // I'm assuming we are using a linear relationship for the distance and the angle

    public void runFromDistance(DoubleSupplier distanceMeters) {
        m_desiredAngleRad = () -> kSlope * distanceMeters.getAsDouble() + kIntercept;
    }

    public void stop() {
        m_hasDesiredAngle = false;
        m_io.setPosition(m_inputs.positionRad);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(kLogPath, m_inputs);
        
        if(DriverStation.isDisabled()) {
            m_io.setPosiiont(m_inputs.positionRad);
            return;
        }

        if(m_hasDesiredAngle) {
            double desired = m_desiredAngleRad.getAsDouble();

            desired = MathUtil.clamp(desired, kMinAngle.getRadians(), kMaxAngle.getRadians());

            Logger.recordOutput(kLogPath + "/DesiredAngle", Math.toDegrees(desired));
        }

        RobotState.getInstance().updateHood(m_inputs.positionRad);
    }
}
