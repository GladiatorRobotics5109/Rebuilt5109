package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;

import static frc.robot.Constants.FlywheelsConstants.*;

import java.util.function.DoubleSupplier;

public class FlywheelsCommands {
    public static Command autoAim(FlywheelsSubsystem flywheels) {
        return Commands.startEnd(
            () -> flywheels.runVelocity(() -> RobotState.getInstance().getAimingParameters().flywheelsRPM()),
            flywheels::stop,
            flywheels
        ).withName("Flywheels::autoAim");
    }

    public static Command setVelocity(FlywheelsSubsystem flywheels, DoubleSupplier velocity) {
        return Commands.startEnd(() -> flywheels.runVelocity(velocity), flywheels::stop, flywheels);
    }
}
