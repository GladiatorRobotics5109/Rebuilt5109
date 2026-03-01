package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutoCommands {
    private static PathPlannerPath kTestPath;

    public static void init(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer
    ) {
        NamedCommands.registerCommand(
            "StartShoot",
            IndexerCommands.index(indexer).andThen(Commands.print("StartShoot"))
        );
        NamedCommands.registerCommand("StopShoot", IndexerCommands.stop(indexer).andThen(Commands.print("StopShoot")));
        // NamedCommands.registerCommand("Shoot", shoot(indexer).andThen(Commands.print("Shoot")));
        NamedCommands.registerCommand("Shoot", Commands.print("Shoot"));
        // TODO: Add impl
        NamedCommands.registerCommand("StartIntake", Commands.none().andThen(Commands.print("StartIntake")));
        NamedCommands.registerCommand("StopIntake", Commands.none().andThen(Commands.print("StopIntake")));

        try {
            kTestPath = PathPlannerPath.fromChoreoTrajectory("Test");
        }
        catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load PathPlannerPath", e.getStackTrace());
        }
    }

    public static Command test(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer
    ) {
        return Commands.sequence(
            prefix(kTestPath, drive),
            AutoBuilder.followPath(kTestPath)
        ).withName("AutoCommands::test");
    }

    private static Command prefix(PathPlannerPath firstPath, DriveSubsystem drive) {
        return drive.runOnce(() -> drive.setPose(firstPath.getStartingHolonomicPose().orElse(Pose2d.kZero)));
    }

    private static Command shoot(IndexerSubsystem indexer) {
        return Commands.sequence(
            IndexerCommands.index(indexer),
            Commands.waitSeconds(5),
            IndexerCommands.stop(indexer)
        );
    }
}
