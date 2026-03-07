package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lombok.experimental.UtilityClass;

import java.util.function.DoubleSupplier;

@UtilityClass
public class IntakeCommands {
    public Command testRollers(IntakeSubsystem intake, DoubleSupplier rollersVoltage) {
        return intake.runEnd(() -> intake.runRollersVoltage(rollersVoltage.getAsDouble()), intake::stop);
    }

    public Command testPivot(IntakeSubsystem intake, DoubleSupplier pivotVoltage) {
        return intake.runEnd(() -> intake.runPivotVoltage(pivotVoltage.getAsDouble()), intake::stop);
    }

    public Command deploy(IntakeSubsystem intake) {
        return intake.runOnce(intake::deploy);
    }

    public Command stow(IntakeSubsystem intake) {
        return intake.startEnd(intake::stow, intake::stop);
    }
}
