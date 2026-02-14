package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SuperstructureCommands {
    public static Command autoAim(TurretSubsystem turret, FlywheelsSubsystem flywheels) {
        return Commands.parallel(
            TurretCommands.autoAim(turret),
            FlywheelsCommands.autoAim(flywheels)
        );
    }
}
