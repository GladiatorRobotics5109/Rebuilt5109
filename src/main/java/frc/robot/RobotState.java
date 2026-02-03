package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AimingConstants;
import frc.robot.Constants.FlywheelsConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.util.AllianceFlip;
import lombok.AccessLevel;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@Getter
public class RobotState {
    private static RobotState s_instance;

    public static void init() {
        s_instance = new RobotState();
    }

    public static RobotState getInstance() { return s_instance; }

    private FuelStrategy m_fuelStrategy = FuelStrategy.HUB;

    @Getter(AccessLevel.NONE)
    private AimingParameters m_latestAimingParameters;
    private final Debouncer m_idleDebouncer = new Debouncer(
        FlywheelsConstants.kIdleDistDebounce,
        DebounceType.kFalling
    );

    // -- Drive State --

    private Pose2d m_pose;
    private ChassisSpeeds m_velocityFieldRelative;
    private ChassisSpeeds m_velocity;

    public Rotation2d getRotation() { return m_pose.getRotation(); }

    // -- Flywheels State --

    @AutoLogOutput(key = "RobotState/Flywheels/FlywheelsRPM")
    private double m_flywheelsRPM;

    // -- Turret State --

    /** Robot relative turret position */
    private Rotation2d m_turretPosition;
    /** Field relative turret position */
    private Rotation2d m_turretHeading;

    // -- Hood State --
    private Rotation2d m_hoodAngle = HoodConstants.kMaxAngle; // TODO: Change this when hood subsystem is written

    // -- Indexer State --
    private boolean m_indexing;

    public AimingParameters getAimingParameters() {
        if (m_latestAimingParameters != null)
            return m_latestAimingParameters;

        Translation2d target = switch (m_fuelStrategy) {
            case HUB -> AllianceFlip.apply(Hub.topCenterPoint).toTranslation2d();
            case SHUTTLE_AUTO -> {
                if (m_pose.getY() >= LinesHorizontal.center) {
                    yield DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? AimingConstants.kShuttleBlueTop
                        : AimingConstants.kShuttleRedTop;
                }
                else {
                    yield DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? AimingConstants.kShuttleBlueBottom
                        : AimingConstants.kShuttleRedBottom;
                }
            }
        };

        Pose2d predicted = new Pose2d(
            m_pose.getX() + m_velocityFieldRelative.vxMetersPerSecond * AimingConstants.kDriveLookaheadTime,
            m_pose.getY() + m_velocityFieldRelative.vyMetersPerSecond * AimingConstants.kDriveLookaheadTime,
            Rotation2d.fromRadians(
                m_pose.getRotation().getRadians()
                    + m_velocityFieldRelative.omegaRadiansPerSecond * AimingConstants.kDriveLookaheadTime
            )
        );

        Translation2d delta = target.minus(predicted.getTranslation());
        Rotation2d targetPosition = delta.getAngle().minus(predicted.getRotation());
        double dist = delta.getNorm();
        Rotation2d pitch = Rotation2d.fromRadians(
            MathUtil.clamp(
                m_fuelStrategy == FuelStrategy.HUB
                    ? AimingConstants.kHubHoodPitch.get(dist)
                    : AimingConstants.kShuttleHoodPitch.get(dist),
                HoodConstants.kMinAngle.getRadians(),
                HoodConstants.kMaxAngle.getRadians()
            )
        );
        double flywheelsRPM;
        if (m_fuelStrategy == FuelStrategy.HUB) {
            flywheelsRPM = m_idleDebouncer.calculate(dist < FlywheelsConstants.kIdleDistThresholdMeters)
                ? AimingConstants.kHubFlywheelsRPMs.get(dist)
                : FlywheelsConstants.kIdleRPM;
        }
        else {
            flywheelsRPM = AimingConstants.kShuttleFlywheelsRPMs.get(dist);
        }

        m_latestAimingParameters = new AimingParameters(targetPosition, flywheelsRPM, pitch);
        Logger.recordOutput("RobotState/LatestAimingParameters", m_latestAimingParameters);

        return m_latestAimingParameters;
    }

    public void updateDrive(Pose2d pose, ChassisSpeeds velocity) {
        m_latestAimingParameters = null;

        m_pose = pose;
        m_velocity = velocity;
        m_velocityFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, pose.getRotation());
    }

    public void updateFlywheels(double flywheelsRPM) {
        m_flywheelsRPM = flywheelsRPM;
    }

    public void updateTurret(Rotation2d turretPosition) {
        m_turretPosition = turretPosition;

        m_turretHeading = getRotation().plus(m_turretPosition);
    }

    public void updateIndexer(boolean indexing) {
        m_indexing = indexing;
    }

    public void setFuelStrategy(FuelStrategy strategy) {
        m_latestAimingParameters = null;

        m_fuelStrategy = strategy;
    }

    public void log() {
        Logger.recordOutput("RobotState/FuelStrategy", m_fuelStrategy);

        Logger.recordOutput("RobotState/Drive/Pose", m_pose);
        Logger.recordOutput("RobotState/Drive/VelocityFieldRelative", m_velocityFieldRelative);
        Logger.recordOutput("RobotState/Drive/Velocity", m_velocity);

        Logger.recordOutput("RobotState/Flywheels/FlywheelsRPM", m_flywheelsRPM);

        Logger.recordOutput("RobotState/Turret/TurretPosition", m_turretPosition);
        Logger.recordOutput("RobotState/Turret/TurretHeading", m_turretHeading);

        Logger.recordOutput("RobotState/Hood/HoodAngle", m_hoodAngle);

        Logger.recordOutput("RobotState/Indexer/Indexing", m_indexing);
    }

    public enum FuelStrategy {
        HUB,
        SHUTTLE_AUTO
    }

    public record AimingParameters(Rotation2d turretPosition, double flywheelsRPM, Rotation2d hoodAngle) {}
}
