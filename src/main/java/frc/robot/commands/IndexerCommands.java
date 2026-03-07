package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

import static frc.robot.Constants.IndexerConstants.*;

public class IndexerCommands {
    public static Command index(IndexerSubsystem indexer) {
        return indexer.runOnce(() -> indexer.runVoltage(kIndexVoltage));
    }

    public static Command stop(IndexerSubsystem indexer) {
        return indexer.runOnce(indexer::stop);
    }
}
