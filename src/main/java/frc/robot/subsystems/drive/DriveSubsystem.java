// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
        Math.max(
            Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
        ),
        Math.max(
            Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        )
    );

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1)
                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1
        ),
        getModuleTranslations()
    );

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert = new Alert(
        "Disconnected gyro, using kinematics as fallback.",
        AlertType.kError
    );

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        rawGyroRotation,
        lastModulePositions,
        Pose2d.kZero
    );

    public DriveSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                this.gyroIO = new GyroIOPigeon2();
                modules[0] = new Module(new ModuleIOTalonFX(TunerConstants.FrontLeft), 0, TunerConstants.FrontLeft);
                modules[1] = new Module(new ModuleIOTalonFX(TunerConstants.FrontRight), 1, TunerConstants.FrontRight);
                modules[2] = new Module(new ModuleIOTalonFX(TunerConstants.BackLeft), 2, TunerConstants.BackLeft);
                modules[3] = new Module(new ModuleIOTalonFX(TunerConstants.BackRight), 3, TunerConstants.BackRight);

                break;
            case SIM:
                this.gyroIO = new GyroIO() {};
                modules[0] = new Module(new ModuleIOSim(TunerConstants.FrontLeft), 0, TunerConstants.FrontLeft);
                modules[1] = new Module(new ModuleIOSim(TunerConstants.FrontRight), 1, TunerConstants.FrontRight);
                modules[2] = new Module(new ModuleIOSim(TunerConstants.BackLeft), 2, TunerConstants.BackLeft);
                modules[3] = new Module(new ModuleIOSim(TunerConstants.BackRight), 3, TunerConstants.BackRight);

                break;
            default:
                this.gyroIO = new GyroIO() {};
                modules[0] = new Module(new ModuleIO() {}, 0, TunerConstants.FrontLeft);
                modules[1] = new Module(new ModuleIO() {}, 1, TunerConstants.FrontRight);
                modules[2] = new Module(new ModuleIO() {}, 2, TunerConstants.BackLeft);
                modules[3] = new Module(new ModuleIO() {}, 3, TunerConstants.BackRight);

                break;
        }

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                Logger.recordOutput("Subsystems/Drive/Trajectory", activePath.toArray(new Pose2d[0]));
            }
        );
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
                Logger.recordOutput("Subsystems/Drive/TrajectorySetpoint", targetPose);
            }
        );

        // Configure SysId
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Subsystems/Drive/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)),
                null,
                this
            )
        );
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Subsystems/Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Subsystems/Drive/ModuleSetpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Subsystems/Drive/ModuleSetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                        - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle
                );
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            }
            else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.kCurrentMode != Mode.SIM);

        RobotState.getInstance().updateDrive(getPose(), getChassisSpeeds());
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Subsystems/Drive/DesiredModuleStates", setpointStates);
        RobotState.getInstance().setDesiredVelocity(discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("Subsystems/Drive/DesiredModuleStatesOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() { return poseEstimator.getEstimatedPosition(); }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() { return getPose().getRotation(); }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds,
            visionMeasurementStdDevs
        );

        Pose3d tag = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(13).get();
        Logger.recordOutput("DeltaX", tag.getX() - visionRobotPoseMeters.getX());
        Logger.recordOutput("DeltaY", tag.getY() - visionRobotPoseMeters.getY());
        Logger.recordOutput(
            "DeltaRot",
            Units.radiansToDegrees(tag.getRotation().getZ() - visionRobotPoseMeters.getRotation().getRadians())
        );
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
}
