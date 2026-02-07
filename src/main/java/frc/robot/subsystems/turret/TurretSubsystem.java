package frc.robot.subsystems.turret;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    private TurretPoseEstimator m_poseEstimator = new TurretPoseEstimator(
        kRobotToTurret,
        Pose3d.kZero,
        new ChassisSpeeds()
    );
    private Supplier<Pose2d> m_robotPose;
    private Supplier<ChassisSpeeds> m_robotSpeeds;

    public TurretSubsystem() {
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new TurretIOTalonFX(kId, Constants.kCANBusCANivore);
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

    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
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

        m_poseEstimator.update(
            m_robotPose.get(),
            m_robotSpeeds.get(),
            Rotation2d.fromRadians(m_inputs.positionRad),
            Rotation2d.fromRadians(m_inputs.velocityRadPerSec)
        );

        RobotState.getInstance().updateTurret(Rotation2d.fromRadians(m_inputs.positionRad));
    }
}
