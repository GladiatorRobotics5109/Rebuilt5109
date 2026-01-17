package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Hub;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheels.Flywheels;

import static frc.robot.Constants.FlywheelsConstants.*;

public class FlywheelsCommands {
    public static Command spin(Flywheels flywheels) {
        return Commands.runOnce(() -> flywheels.runVelocity(kShootRPM), flywheels).withName("Flywheels::spin");
    }

    public static Command auto(Flywheels flywheels) {
        Debouncer distDebounce = new Debouncer(kIdleDistDebounce, DebounceType.kFalling);

        return Commands.run(() -> {
            Translation2d hub = Hub.hubCenterPositionFlipped();
            Pose2d robot = RobotState.getPose();
            double dist = robot.getTranslation().getDistance(hub);

            if (distDebounce.calculate(dist < kIdleDistThresholdMeters)) {
                flywheels.runVelocity(kIdleRPM);
            }
            else {
                flywheels.stop();
            }

        }, flywheels).withName("Flywheels::auto");
    }
}
