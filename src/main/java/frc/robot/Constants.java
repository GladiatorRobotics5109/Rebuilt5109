// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;
    public static final boolean kTuningMode = true;

    public static final CANBus kCANBusRio = CANBus.roboRIO();
    public static final CANBus kCANBusCANivore = new CANBus("CANivore");

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

        public static final int kId = 0;
        public static final double kStatorCurrentLimit = 0.0;
        public static final boolean kStatorCurrentLimitEnable = false;
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentLimitEnable = true;
        public static final double kGearRatio = 1.0;
        public static final boolean kInverted = false;

        public static final double kShootRPM = 6000;
        public static final double kIdleRPM = 1000;

        public static final double kIdleDistThresholdMeters = 5.0;
        public static final double kIdleDistDebounce = 0.5;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class VisionConstants {
        public static final String kCamera1Name = "Camera1";
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
