package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import lombok.experimental.UtilityClass;

@UtilityClass
public class AutoCommands {
    public void init(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelsSubsystem flywheels,
        IndexerSubsystem indexer
    ) {
        NamedCommands.registerCommand("Shoot", IndexerCommands.index(indexer));
    }
}
