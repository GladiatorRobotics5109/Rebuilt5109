package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlywheelsConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.util.fuelsim.FuelSim;
import org.littletonrobotics.junction.Logger;

public class Visualizer {
    private static Visualizer s_instance;

    public static void init(DriveSubsystem drive, FlywheelsSubsystem flywheels) {
        s_instance = new Visualizer(drive, flywheels);
    }

    public static Visualizer getInstance() { return s_instance; }

    private Visualizer(DriveSubsystem drive, FlywheelsSubsystem flywheels) {
        // FuelSim.getInstance().spawnStartingFuel();
        FuelSim.getInstance().registerRobot(
            Units.inchesToMeters(28),
            Units.inchesToMeters(24),
            Units.inchesToMeters(5.25),
            drive::getPose,
            drive::getChassisSpeeds
        );
        FuelSim.getInstance().start();

        new Trigger(RobotState.getInstance()::isIndexing).whileTrue(Commands.repeatingSequence(Commands.runOnce(() -> {
            Pose2d robot = RobotState.getInstance().getPose();
            ChassisSpeeds speeds = RobotState.getInstance().getVelocityFieldRelative();
            double flywheelsRPM = RobotState.getInstance().getFlywheelsRPM();
            Rotation2d turretHeading = RobotState.getInstance().getTurretPosition();
            Rotation2d shooterAngle = RobotState.getInstance().getHoodAngle();

            FuelSim.getInstance().spawnFuel(
                getTurretPose().getTranslation(),
                new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0).plus(
                    new Translation3d(
                        FlywheelsConstants.kSimShooterEfficiency
                            * FlywheelsConstants.kSimShooterWheelRadius
                            * Units.rotationsPerMinuteToRadiansPerSecond(flywheelsRPM),
                        new Rotation3d(0.0, -shooterAngle.getRadians(), turretHeading.getRadians())
                    )
                )
            );
            flywheels.simulateShot();
        }), Commands.waitSeconds(1 / FlywheelsConstants.kSimShootRate)));
    }

    public void periodic() {
        FuelSim.getInstance().updateSim();

        Logger.recordOutput("Visualizer/TurretPosition", getTurretPose());
    }

    private Pose3d getTurretPose() {
        Pose2d robot = RobotState.getInstance().getPose();
        return new Pose3d(
            new Translation3d(robot.getX(), robot.getY(), 0).plus(
                TurretConstants.kRobotToTurret.getTranslation().rotateBy(
                    new Rotation3d(0, 0, robot.getRotation().getRadians())
                )
            ),
            new Rotation3d(
                0,
                -RobotState.getInstance().getHoodAngle().getRadians(),
                RobotState.getInstance().getTurretHeading().getRadians()
            )
        );
    }
}
