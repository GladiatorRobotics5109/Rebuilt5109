// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.util.AllianceFlip;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;
    public static final boolean kTuningMode = true;
    public static final boolean kSimShouldUseKeyboard = kCurrentMode == Mode.SIM && false;

    public static final CANBus kCANBusRio = CANBus.roboRIO();
    public static final CANBus kCANBusCANivore = new CANBus("drivetrain");

    public static final class DriveCommandsConstants {
        public static final double kDeadband = 0.1;
        public static final double kAngleP = 5.0;
        public static final double kAngleD = 0.4;
        public static final double kAngleMaxVelocity = 8.0;
        public static final double kAngleMaxAcceleration = 20.0;
        public static final double kFFStartDelay = 2.0; // Secs
        public static final double kFFRampRate = 0.1; // Volts/Sec
        public static final double kWheelRadiusMaxVelocity = 0.25; // Rad/Sec
        public static final double kWheelRadiusRampRate = 0.05; // Rad/Sec^2
    }

    public static final class FlywheelsConstants {
        public static final String kLogPath = "Subsystems/Flywheels";

        public static final int kId = 4;
        public static final double kStatorCurrentLimit = 0.0;
        public static final boolean kStatorCurrentLimitEnable = false;
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentLimitEnable = false;
        public static final double kGearRatio = 1.0;
        public static final boolean kInverted = true;

        public static final double kShootRPM = 5500;
        public static final double kIdleRPM = 1000;

        public static final double kIdleDistThresholdMeters = 5.0;
        public static final double kIdleDistDebounce = 0.5;

        public static final double kS = 0.19;
        public static final double kV = 0.018;
        public static final double kA = 0.0;
        public static final double kBangBangTolerance = 75;

        public static final double kSimMOI = 0.0004475;

        public static final double kSimShooterWheelRadius = Units.inchesToMeters(2);
        public static final double kSimShooterEfficiency = 0.30;
        public static final double kSimShootRate = 5;
    }

    public static final class TurretConstants {
        public static final String kLogPath = "Subsystems/Turret";

        public static final int kId = 0;
        public static final double kStatorCurrentLimit = 0.0;
        public static final boolean kStatorCurrentLimitEnable = false;
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentLimitEnable = true;
        public static final double kGearRatio = 1.0;
        public static final boolean kInverted = false;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final Transform3d kRobotToTurret = new Transform3d(
            Units.inchesToMeters(7.247244),
            0.0,
            Units.inchesToMeters(13.375000),
            Rotation3d.kZero
        );
    }

    public static final class HoodConstants {
        public static final String kLogPath = "Subsystems/Hood";

        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(20);
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(55);
    }

    public static final class IndexerConstants {
        public static final String kLogPath = "Subsystems/Indexer";

        public static final double kIndexVoltage = 12.0;
    }

    public static final class VisionConstants {
        public static final String kCamera1Name = "limelight-one";

        public static final AprilTagFieldLayout kAprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();;

        // Robot to camera transforms
        public static Transform3d kTurretToCamera1 = new Transform3d(
            Units.inchesToMeters(7.225597),
            0.0,
            Units.inchesToMeters(3.168515),
            new Rotation3d(0.0, Units.degreesToRadians(-10), 0.0)
        );

        // Basic filtering thresholds
        public static final double kMaxAmbiguity = 0.3;
        public static final double kMaxZError = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static final double kLinearStdDevBaseline = 0.02; // Meters
        public static final double kAngularStdDevBaseline = 0.06; // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static final double[] kCameraStdDevFactors = new double[] {
            1.0, // Camera 0
        };

        // Multipliers to apply for MegaTag 2 observations
        public static final double kLinearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static final double kAngularStdDevMegatag2Factor = 0.5; // No rotation data available
        // public static final double kAngularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
    }

    public static final class AimingConstants {
        public static final double kDriveLookaheadTime = 0.1;

        public static final InterpolatingDoubleTreeMap kHubFlywheelsRPMs = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap kShuttleFlywheelsRPMs = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap kHubHoodPitch = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap kShuttleHoodPitch = new InterpolatingDoubleTreeMap();

        static {
            kHubFlywheelsRPMs.put(1.0, 3000.0);
            kHubFlywheelsRPMs.put(6.0, 5500.0);

            kShuttleFlywheelsRPMs.put(1.0, 5500.0);

            kHubHoodPitch.put(1.0, HoodConstants.kMaxAngle.getRadians());
            kShuttleHoodPitch.put(1.0, HoodConstants.kMinAngle.getRadians());
        }

        public static final Translation2d kShuttleBlueTop = new Translation2d(
            LinesVertical.allianceZone / 2,
            LinesHorizontal.center / 4 * 3
        );

        public static final Translation2d kShuttleBlueBottom = new Translation2d(
            LinesVertical.allianceZone / 2,
            LinesHorizontal.center / 4
        );

        public static final Translation2d kShuttleRedTop = new Translation2d(
            AllianceFlip.flipX(kShuttleBlueTop.getX()),
            LinesHorizontal.center / 4
        );

        public static final Translation2d kShuttleRedBottom = new Translation2d(
            AllianceFlip.flipX(kShuttleBlueBottom.getX()),
            LinesHorizontal.center / 4 * 3
        );
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
