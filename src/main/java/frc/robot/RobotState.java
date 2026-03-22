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

import lombok.Setter;
import lombok.experimental.Accessors;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

@Getter
public class RobotState {
    private static RobotState s_instance;

    public static void init() {
        s_instance = new RobotState();

        // Log empty AimingParameters on initialization to avoid loop overruns
        Logger.recordOutput(
            "RobotState/AimingParameters/LatestAimingParameters",
            new AimingParameters(Rotation2d.kZero, 0.0, Rotation2d.kZero)
        );
    }

    public static RobotState getInstance() { return s_instance; }

    private FuelStrategy m_fuelStrategy = FuelStrategy.HUB;

    @Getter(AccessLevel.NONE)
    private AimingParameters m_latestAimingParameters;
    @Getter(AccessLevel.NONE)
    private final Debouncer m_idleDebouncer = new Debouncer(
        FlywheelsConstants.kIdleDistDebounce,
        DebounceType.kFalling
    );

    // -- Drive State --

    private Pose2d m_pose = Pose2d.kZero;
    private ChassisSpeeds m_velocityFieldRelative = new ChassisSpeeds();
    private ChassisSpeeds m_velocity = new ChassisSpeeds();
    @Setter
    private ChassisSpeeds m_desiredVelocity = new ChassisSpeeds();

    public Rotation2d getRotation() { return m_pose.getRotation(); }

    // -- Flywheels State --

    private double m_flywheelsVelocity;
    private double m_flywheelsDesiredVelocity;
    private boolean m_flywheelsHasDesiredVelocity;
    @Setter
    @Accessors(fluent = true)
    private boolean m_flywheelsShouldDisableAutoAim;

    // -- Turret State --

    /** Robot relative turret position */
    private Rotation2d m_turretPosition;
    /** Field relative turret position */
    private Rotation2d m_turretHeading;
    private double m_turretVelocityRadPerSec;

    // -- Hood State --
    private Rotation2d m_hoodAngle;

    // -- Indexer State --
    private boolean m_indexing;

    public AimingParameters getAimingParameters() {
        if (m_latestAimingParameters != null)
            return m_latestAimingParameters;

        Translation2d target = switch (m_fuelStrategy) {
            case HUB -> AllianceFlip.apply(Hub.topCenterPoint).toTranslation2d();
            case SHUTTLE -> {
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
            boolean shouldIdle = shouldIdle();
            Logger.recordOutput("AimingParameters/ShouldIdle", shouldIdle);
            flywheelsRPM = shouldIdle
                ? FlywheelsConstants.kIdleRPM
                : AimingConstants.kHubFlywheelsRPMs.get(dist);
        }
        else {
            flywheelsRPM = AimingConstants.kShuttleFlywheelsRPMs.get(dist);
        }

        m_latestAimingParameters = new AimingParameters(targetPosition, flywheelsRPM, pitch);
        Logger.recordOutput("AimingParameters/LatestAimingParameters", m_latestAimingParameters);
        Logger.recordOutput("AimingParameters/Target", target);
        Logger.recordOutput("AimingParameters/Predicted", predicted);
        Logger.recordOutput("AimingParameters/Delta", delta);
        Logger.recordOutput("AimingParameters/Dist", dist);

        return m_latestAimingParameters;
    }

    private boolean shouldIdle() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            return false;

        return m_idleDebouncer.calculate(switch (alliance.get()) {
            case Blue -> m_pose.getX()
                > FieldConstants.LinesVertical.allianceZone + FieldConstants.LinesVertical.neutralZoneNear;
            case Red -> m_pose.getX() < FieldConstants.LinesVertical.neutralZoneFar;
        });
    }

    public void updateDrive(Pose2d pose, ChassisSpeeds velocity) {
        m_latestAimingParameters = null;

        m_pose = pose;
        m_velocity = velocity;
        m_velocityFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, pose.getRotation());

    }

    public void updateFlywheels(double velocity, double desiredVelocity, boolean hasDesiredVelocity) {
        m_flywheelsVelocity = velocity;
        m_flywheelsDesiredVelocity = desiredVelocity;
        m_flywheelsHasDesiredVelocity = hasDesiredVelocity;
    }

    public void updateTurret(Rotation2d turretPosition, double turretVelocityRadPerSec) {
        m_turretPosition = turretPosition;

        m_turretHeading = getRotation().plus(m_turretPosition);

        m_turretVelocityRadPerSec = turretVelocityRadPerSec;
    }

    public void updateHood(Rotation2d hoodAngle) {
        m_hoodAngle = hoodAngle;
    }

    public void updateIndexer(boolean indexing) {
        m_indexing = indexing;
    }

    public void setFuelStrategy(FuelStrategy strategy) {
        m_latestAimingParameters = null;

        m_fuelStrategy = strategy;
    }

    public void log() {
        Logger.recordOutput("FuelStrategy", m_fuelStrategy);

        Logger.recordOutput("Subsystems/Drive/Pose", m_pose);
        Logger.recordOutput("Subsystems/Drive/VelocityFieldRelative", m_velocityFieldRelative);
        Logger.recordOutput("Subsystems/Drive/Velocity", m_velocity);
        Logger.recordOutput("Subsystems/Drive/DesiredVelocity", m_desiredVelocity);

        Logger.recordOutput("Subsystems/Flywheels/CurrentVelocity", m_flywheelsVelocity);
        Logger.recordOutput("Subsystems/Flywheels/DesiredVelocity", m_flywheelsDesiredVelocity);
        Logger.recordOutput("Subsystems/Flywheels/HasDesiredVeloicty", m_flywheelsHasDesiredVelocity);

        Logger.recordOutput("Subsystems/Turret/TurretPosition", m_turretPosition);
        Logger.recordOutput("Subsystems/Turret/TurretHeading", m_turretHeading);

        Logger.recordOutput("Subsystems/Hood/HoodAngle", m_hoodAngle);

        Logger.recordOutput("Subsystems/Indexer/Indexing", m_indexing);
    }

    public enum FuelStrategy {
        HUB,
        SHUTTLE
    }

    public record AimingParameters(Rotation2d turretPosition, double flywheelsRPM, Rotation2d hoodAngle) {}
}
