package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.HoodSubsystem;
import lombok.experimental.UtilityClass;

@UtilityClass
public class HoodCommands {
    public Command stow(HoodSubsystem hood) {
        // TODO: add impl
        return Commands.print("Stow hood").withName("HoodCommands::stow");
    }
}
