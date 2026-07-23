package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlywheelsConstants;
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
        FuelSim.getInstance().spawnStartingFuel();
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
            double flywheelsRPM = RobotState.getInstance().getFlywheelsVelocity();
            Pose3d turret = getTurretPose();

            FuelSim.getInstance().spawnFuel(
                turret.getTranslation(),
                new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0).plus(
                    new Translation3d(
                        FlywheelsConstants.kSimShooterEfficiency
                            * FlywheelsConstants.kSimShooterWheelRadius
                            * Units.rotationsPerMinuteToRadiansPerSecond(flywheelsRPM),
                        turret.getRotation()
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
        RobotState state = RobotState.getInstance();
        return Conversions.robotPoseToTurretPose(state.getPose(), state.getTurretPosition(), state.getHoodAngle());
    }
}
