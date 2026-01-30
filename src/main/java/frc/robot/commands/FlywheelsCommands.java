package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;

import static frc.robot.Constants.FlywheelsConstants.*;

public class FlywheelsCommands {
    public static Command spin(FlywheelsSubsystem flywheels) {
        return Commands.runOnce(() -> flywheels.runVelocity(kShootRPM), flywheels).withName("Flywheels::spin");
    }

    public static Command autoAim(FlywheelsSubsystem flywheels) {
        Debouncer distDebounce = new Debouncer(kIdleDistDebounce, DebounceType.kFalling);

        return Commands.run(
            () -> flywheels.runVelocity(RobotState.getInstance().getAimingParameters().flywheelsRPM()),
            flywheels
        ).withName("Flywheels::autoAim");
    }
}
