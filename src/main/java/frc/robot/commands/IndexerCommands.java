package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FlywheelsConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.util.fuelsim.FuelSim;

import static frc.robot.Constants.IndexerConstants.*;

public class IndexerCommands {
    public static Command index(IndexerSubsystem indexer) {
        return indexer.startEnd(() -> indexer.runVoltage(kIndexVoltage), indexer::stop);
    }

    public static Command stop(IndexerSubsystem indexer) {
        return indexer.runOnce(indexer::stop);
    }

    public static Command indexSim(IndexerSubsystem indexer, FlywheelsSubsystem flywheels) {
        return index(indexer).alongWith(
            Commands.repeatingSequence(
                Commands.runOnce(() -> {
                    Pose2d robot = RobotState.getInstance().getPose();
                    double flywheelsRPM = RobotState.getInstance().getFlywheelsRPM();
                    Rotation2d turretHeading = RobotState.getInstance().getTurretPosition();
                    Rotation2d shooterAngle = RobotState.getInstance().getHoodAngle();

                    FuelSim.getInstance().spawnFuel(
                        new Translation3d(
                            robot.getTranslation().getX(),
                            robot.getTranslation().getY(),
                            Units.inchesToMeters(14.8)
                        ),
                        new Translation3d(
                            FlywheelsConstants.kSimShooterEfficiency
                                * FlywheelsConstants.kSimShooterWheelRadius
                                * Units.rotationsPerMinuteToRadiansPerSecond(flywheelsRPM),
                            new Rotation3d(0.0, -shooterAngle.getRadians(), turretHeading.getRadians())
                        )
                    );
                    flywheels.simulateShot();
                }),
                Commands.waitSeconds(1 / FlywheelsConstants.kSimShootRate)
            )
        );
    }
}
