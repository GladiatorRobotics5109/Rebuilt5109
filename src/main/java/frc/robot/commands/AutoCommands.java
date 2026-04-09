package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AimingConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.FieldConstants.Outpost;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.Flip;
import frc.robot.util.Conversions;
import frc.robot.util.LoggedTunableNumber;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoCommands {
    // private static PathPlannerPath kTestPath;
    private static PathPlannerPath kPreloadAndOutpostPath;
    private static PathPlannerPath kPreloadAndDepotPath;
    private static PathPlannerPath kPreloadAndCenterLeft;
    private static PathPlannerPath kPreloadAndCenterRight;
    private static PathPlannerPath kNewRight1;
    private static PathPlannerPath kNewRight2;
    private static PathPlannerPath kNewCenter1;
    private static PathPlannerPath kNewCenter2;

    private static PathPlannerPath kNewRight1Mirrored;
    private static PathPlannerPath kNewRight2Mirrored;
    private static PathPlannerPath kNewCenter1Mirrored;
    private static PathPlannerPath kNewCenter2Mirrored;

    private static final LoggedTunableNumber s_flywheelsVelocity = new LoggedTunableNumber(
        "TestFlywheelsVelocityRPM",
        0.0
    );
    private static final LoggedTunableNumber s_turretPosition = new LoggedTunableNumber("TestTurretPositionDeg", 0.0);

    private static DoubleSupplier s_preAutoDelay;
    private static BooleanSupplier s_isRight;

    public static void init(DoubleSupplier preAutoDelay, BooleanSupplier isRight) {
        s_preAutoDelay = preAutoDelay;
        s_isRight = isRight;

        try {
            kPreloadAndOutpostPath = PathPlannerPath.fromChoreoTrajectory("PreloadAndOutpost");
            kPreloadAndDepotPath = PathPlannerPath.fromChoreoTrajectory("PreloadAndDepot");
            kPreloadAndCenterLeft = PathPlannerPath.fromChoreoTrajectory("PreloadAndCenterLeft");
            kPreloadAndCenterRight = PathPlannerPath.fromChoreoTrajectory("PreloadAndCenterRight");
            kNewRight1 = PathPlannerPath.fromChoreoTrajectory("NewRight_1");
            kNewRight2 = PathPlannerPath.fromChoreoTrajectory("NewRight_2");
            kNewCenter1 = PathPlannerPath.fromChoreoTrajectory("NewCenter_1");
            kNewCenter2 = PathPlannerPath.fromChoreoTrajectory("NewCenter_2");

            kNewRight1Mirrored = kNewRight1.mirrorPath();
            kNewRight2Mirrored = kNewRight2.mirrorPath();
            kNewCenter1Mirrored = kNewCenter1.mirrorPath();
            kNewCenter2Mirrored = kNewCenter2.mirrorPath();
        }
        catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load PathPlannerPath", e.getStackTrace());
        }
    }

    public static Command preloadLeft(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            prefix(
                () -> Flip.apply(
                    new Pose2d(
                        LinesVertical.starting - Conversions.inchesToMeters(12),
                        LinesHorizontal.leftTrenchOpenStart - Conversions.inchesToMeters(16 + 3),
                        Rotation2d.kCW_Pi_2
                    )
                ),
                () -> Rotation2d.kZero,
                drive,
                turret
            ),
            TurretCommands.leftTrench(turret),
            FlywheelsCommands.runVelocity(flywheels, () -> AimingConstants.kTrenchFlywheelsVelocityRPM),
            Commands.waitSeconds(2),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(4.5),
            IndexerCommands.reverse(indexer),
            Commands.waitSeconds(0.5),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(5),
            IndexerCommands.stop(indexer)
        );
    }

    public static Command preloadRight(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flyhweels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            prefix(
                () -> Flip.apply(
                    new Pose2d(
                        LinesVertical.starting - Conversions.inchesToMeters(12),
                        LinesHorizontal.rightTrenchOpenStart - Conversions.inchesToMeters(16 + 3),
                        Rotation2d.kCCW_Pi_2
                    )
                ),
                () -> Rotation2d.kZero,
                drive,
                turret
            ),
            TurretCommands.rightTrench(turret),
            FlywheelsCommands.runVelocity(flyhweels, () -> AimingConstants.kTrenchFlywheelsVelocityRPM),
            Commands.waitSeconds(2),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(4.5),
            IndexerCommands.reverse(indexer),
            Commands.waitSeconds(0.5),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(5),
            IndexerCommands.stop(indexer)
        );
    }

    public static Command preloadAndOutpostRight(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            prefix(kPreloadAndOutpostPath, Rotation2d.kZero, drive, turret),
            TurretCommands.rightTrench(turret),
            FlywheelsCommands.runVelocity(flywheels, () -> AimingConstants.kTrenchFlywheelsVelocityRPM),
            Commands.waitSeconds(2),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(2.75),
            IndexerCommands.reverse(indexer),
            Commands.waitSeconds(0.5),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(2.75),
            IndexerCommands.stop(indexer),
            IntakeCommands.deploy(intake),
            AutoBuilder.followPath(kPreloadAndOutpostPath),
            Commands.parallel(
                DriveCommands.driveToPose(
                    () -> Flip.apply(
                        new Pose2d(
                            Outpost.centerPoint.plus(new Translation2d(Conversions.inchesToMeters(28), 0.0)),
                            Rotation2d.kZero
                        )
                    ),
                    drive
                ),
                TurretCommands.outpost(turret),
                IndexerCommands.index(indexer).beforeStarting(Commands.waitSeconds(1))
            )
        ).withName("AutoCommands::preloadAndOutpostRight");
    }

    public static Command preloadAndDepotLeft(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            prefix(kPreloadAndDepotPath, Rotation2d.kZero, drive, turret),
            TurretCommands.leftTrench(turret),
            FlywheelsCommands.runVelocity(flywheels, () -> AimingConstants.kTrenchFlywheelsVelocityRPM),
            Commands.waitSeconds(2),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(2.75),
            IndexerCommands.reverse(indexer),
            Commands.waitSeconds(0.5),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(2.75),
            IndexerCommands.stop(indexer),
            Commands.parallel(
                AutoBuilder.followPath(kPreloadAndDepotPath),
                IntakeCommands.deploy(intake).beforeStarting(Commands.waitSeconds(0.8)),
                TurretCommands.autoAim(turret),
                FlywheelsCommands.autoAim(flywheels),
                IndexerCommands.index(indexer).beforeStarting(Commands.waitSeconds(1.2))
            )
        ).withName("AutoCommands::preloadAndDepotLeft");
    }

    public static Command preloadAndCenterLeft(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            prefix(kPreloadAndCenterLeft, Rotation2d.kZero, drive, turret),
            TurretCommands.leftTrench(turret),
            FlywheelsCommands.runVelocity(flywheels, () -> AimingConstants.kTrenchFlywheelsVelocityRPM),
            Commands.waitSeconds(0.5),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(2.25),
            IndexerCommands.stop(indexer),
            Commands.parallel(
                AutoBuilder.followPath(kPreloadAndCenterLeft),
                Commands.sequence(
                    IntakeCommands.deploy(intake).beforeStarting(Commands.waitSeconds(1.5)),
                    IntakeCommands.stow(intake).beforeStarting(Commands.waitSeconds(11))
                ),
                TurretCommands.autoAim(turret),
                FlywheelsCommands.autoAim(flywheels),
                Commands.sequence(
                    Commands.waitSeconds(11.2),
                    Commands.repeatingSequence(
                        IndexerCommands.index(indexer),
                        Commands.waitSeconds(4),
                        IndexerCommands.reverse(indexer),
                        Commands.waitSeconds(1)
                    )
                )
            )
        ).withName("AutoCommands::preloadAndCenterLeft");
    }

    public static Command preloadAndCenterRight(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            // prefix(
            //     () -> Flip.apply(Flip.flipY(kPreloadAndCenterLeft.getStartingHolonomicPose().orElse(Pose2d.kZero))),
            //     () -> Rotation2d.kZero,
            //     drive,
            //     turret
            // ),
            prefix(kPreloadAndCenterRight, Rotation2d.kZero, drive, turret),
            TurretCommands.rightTrench(turret),
            FlywheelsCommands.runVelocity(flywheels, () -> AimingConstants.kTrenchFlywheelsVelocityRPM),
            Commands.waitSeconds(0.5),
            IndexerCommands.index(indexer),
            Commands.waitSeconds(2.25),
            IndexerCommands.stop(indexer),
            Commands.parallel(
                AutoBuilder.followPath(kPreloadAndCenterRight),
                Commands.sequence(
                    IntakeCommands.deploy(intake).beforeStarting(Commands.waitSeconds(1.5)),
                    IntakeCommands.stow(intake).beforeStarting(Commands.waitSeconds(11))
                ),
                TurretCommands.autoAim(turret),
                FlywheelsCommands.autoAim(flywheels),
                Commands.sequence(
                    Commands.waitSeconds(11.3),
                    Commands.repeatingSequence(
                        IndexerCommands.index(indexer),
                        Commands.waitSeconds(4),
                        IndexerCommands.reverse(indexer),
                        Commands.waitSeconds(1)
                    )
                )
            )
        ).withName("AutoCommands::preloadAndCenterRight");
    }

    public static Command newTrench(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            Commands.either(
                prefix(kNewRight1, Rotation2d.kZero, drive, turret),
                prefix(kNewRight1Mirrored, Rotation2d.kZero, drive, turret),
                s_isRight
            ),
            Commands.parallel(
                TurretCommands.autoAim(turret),
                FlywheelsCommands.autoAim(flywheels),
                Commands.sequence(
                    Commands.parallel(
                        Commands.either(
                            AutoBuilder.followPath(kNewRight1),
                            AutoBuilder.followPath(kNewRight1Mirrored),
                            s_isRight
                        ),
                        Commands.sequence(
                            Commands.waitSeconds(0.91),
                            IntakeCommands.deploy(intake),
                            Commands.waitSeconds(11.5),
                            IntakeCommands.stow(intake)
                        ),
                        Commands.sequence(
                            Commands.waitSeconds(8.5),
                            Commands.repeatingSequence(
                                IndexerCommands.index(indexer),
                                Commands.waitSeconds(2),
                                IndexerCommands.reverse(indexer),
                                Commands.waitSeconds(1)
                            ).raceWith(Commands.waitSeconds(3.5))
                        )
                    ),
                    IndexerCommands.stop(indexer),
                    Commands.parallel(
                        Commands.either(
                            AutoBuilder.followPath(kNewRight2),
                            AutoBuilder.followPath(kNewRight2Mirrored),
                            s_isRight
                        ),
                        Commands.sequence(
                            Commands.waitSeconds(1.7),
                            IntakeCommands.deploy(intake),
                            Commands.waitSeconds(10.6),
                            IntakeCommands.stow(intake)
                        ),
                        Commands.sequence(
                            Commands.waitSeconds(9.6),
                            IndexerCommands.index(indexer)
                        )
                    )
                )
            )
        ).withName("AutoCommands::newTrench");
    }

    public static Command newCenter(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer,
        IntakeSubsystem intake
    ) {
        return Commands.sequence(
            Commands.either(
                prefix(kNewCenter1, Rotation2d.kZero, drive, turret),
                prefix(kNewCenter1Mirrored, Rotation2d.kZero, drive, turret),
                s_isRight
            ),
            Commands.parallel(
                TurretCommands.autoAim(turret),
                FlywheelsCommands.autoAim(flywheels),
                Commands.sequence(
                    Commands.parallel(
                        Commands.either(
                            AutoBuilder.followPath(kNewCenter1),
                            AutoBuilder.followPath(kNewCenter1Mirrored),
                            s_isRight
                        ),
                        Commands.sequence(
                            Commands.waitSeconds(0.8),
                            IndexerCommands.index(indexer),
                            Commands.waitSeconds(2),
                            IndexerCommands.stop(indexer)
                        )
                    ),
                    Commands.parallel(
                        Commands.either(
                            AutoBuilder.followPath(kNewCenter2),
                            AutoBuilder.followPath(kNewCenter2Mirrored),
                            s_isRight
                        ),
                        Commands.sequence(
                            Commands.waitSeconds(7),
                            Commands.repeatingSequence(
                                IndexerCommands.index(indexer),
                                Commands.waitSeconds(2),
                                IndexerCommands.reverse(indexer),
                                Commands.waitSeconds(0.5)
                            )
                        ),
                        Commands.sequence(
                            Commands.waitSeconds(1.1),
                            IntakeCommands.deploy(intake),
                            Commands.waitSeconds(7.9),
                            IntakeCommands.stow(intake)
                        )
                    )
                )
            )
        ).withName("AutoCommands::newCenter");
    }

    public static Command test(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer
    ) {
        // return Commands.sequence(
        //     prefix(kTestPath, Rotation2d.kZero, drive, turret),
        //     AutoBuilder.followPath(kTestPath)
        // ).withName("AutoCommands::test");

        return Commands.none().withName("AutoCommands::test");
    }

    public static Command testTurret(FlywheelsSubsystem flywheels, TurretSubsystem turret, IndexerSubsystem indexer) {
        return Commands.runEnd(() -> {
            flywheels.runVelocity(s_flywheelsVelocity::get);
            turret.runPosition(() -> Rotation2d.fromDegrees(s_turretPosition.get()));
            indexer.runVoltage(IndexerConstants.kIndexerIndexVoltage, IndexerConstants.kKickupIndexVoltage);
        },
            () -> {
                flywheels.stop();
                turret.stop();
                indexer.stop();
            }
        );
    }

    private static Command prefix(
        PathPlannerPath firstPath,
        Rotation2d turretPosition,
        DriveSubsystem drive,
        TurretSubsystem turret
    ) {
        return prefix(
            () -> Flip.apply(firstPath.getStartingHolonomicPose().orElse(Pose2d.kZero)),
            () -> turretPosition,
            drive,
            turret
        );
    }

    private static Command prefix(
        Supplier<Pose2d> startingPose,
        Supplier<Rotation2d> turretPosition,
        DriveSubsystem drive,
        TurretSubsystem turret
    ) {
        return Commands.sequence(Commands.runOnce(() -> {
            drive.setPose(startingPose.get());
            turret.setPosition(turretPosition.get());
        }, drive, turret), wait(s_preAutoDelay));
    }

    private static Command wait(DoubleSupplier seconds) {
        Timer timer = new Timer();
        return Commands.startEnd(() -> {
            timer.reset();
            timer.start();
        }, timer::stop).onlyWhile(() -> !timer.hasElapsed(seconds.getAsDouble()));
    }
}
