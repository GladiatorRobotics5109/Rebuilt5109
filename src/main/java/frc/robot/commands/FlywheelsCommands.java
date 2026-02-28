package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;

import java.util.function.DoubleSupplier;

public class FlywheelsCommands {
    public static Command autoAim(FlywheelsSubsystem flywheels) {
        return flywheels.startEnd(
            () -> flywheels.runVelocity(() -> RobotState.getInstance().getAimingParameters().flywheelsRPM()),
            flywheels::stop
        ).withName("Flywheels::autoAim");
    }

    public static Command runVelocity(FlywheelsSubsystem flywheels, DoubleSupplier velocity) {
        return flywheels.startEnd(() -> flywheels.runVelocity(velocity), flywheels::stop).withName(
            "FlywheelsCommands::runVelocity"
        );
    }
}
