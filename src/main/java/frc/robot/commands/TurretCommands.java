package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretCommands {
    public static Command autoAim(TurretSubsystem turret) {
        return Commands.startEnd(
            () -> turret.runPosition(() -> RobotState.getInstance().getAimingParameters().targetHeading()),
            turret::stop,
            turret
        ).withName("TurretCommands::autoAim");
    }
}
