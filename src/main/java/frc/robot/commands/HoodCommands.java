package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.HoodSubsystem;
import lombok.experimental.UtilityClass;

@UtilityClass
public class HoodCommands {
    public Command stow(HoodSubsystem hood) {
        return hood.startEnd(() -> hood.runAngle(HoodConstants.kMaxAngle.getRadians()), hood::stop).withName(
            "HoodCommands::stow"
        );
    }

    public Command autoAim(HoodSubsystem hood) {
        return hood.startEnd(
            () -> hood.runAngle(() -> RobotState.getInstance().getAimingParameters().hoodAngle().getRadians()),
            hood::stop
        ).withName("HoodCommands::autoAim");
    }
}
