package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommands {
    public static Command deploy(IntakeSubsystem intake) { return Commands.none(); }

    public static Command stow(IntakeSubsystem intake) { return Commands.none(); }
}
