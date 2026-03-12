package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.TurretSubsystem;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class TurretCommands {
    public static Command autoAim(TurretSubsystem turret) {
        return turret.startEnd(
            () -> turret.runPosition(() -> RobotState.getInstance().getAimingParameters().turretPosition()),
            turret::stop
        ).withName("TurretCommands::autoAim");
    }

    public static Command straight(TurretSubsystem turret) {
        return turret.runOnce(() -> turret.runPosition(Rotation2d.kZero)).withName(
            "TurretCommands::straight"
        );
    }

    public static Command rightTrench(TurretSubsystem turret) {
        return turret.runOnce(() -> turret.runPosition(TurretConstants.kRightTrenchPosition));
    }

    public static Command leftTrench(TurretSubsystem turret) {
        return turret.runOnce(() -> turret.runPosition(TurretConstants.kLeftTrenchPosition));
    }

    public static Command outpost(TurretSubsystem turret) {
        return turret.runOnce(() -> turret.runPosition(TurretConstants.kOutpostPosition));
    }

    public static Command feedforwardCharacterization(TurretSubsystem turret) {
        final double kFFRampRate = 0.05;

        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }
            ),
            Commands.waitSeconds(0.5),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * kFFRampRate;
                    turret.runVoltage(voltage);
                    velocitySamples.add(turret.getVelocityRadPerSec());
                    voltageSamples.add(voltage);
                },
                turret
            )

                // When canceled, calculate and print results
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
                        double sumX = 0.0;
                        double sumY = 0.0;
                        double sumXY = 0.0;
                        double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += voltageSamples.get(i);
                            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                        }
                        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                        NumberFormat formatter = new DecimalFormat("#0.00000");
                        System.out.println("********** Turret FF Characterization Results **********");
                        System.out.println("\tkS: " + formatter.format(kS));
                        System.out.println("\tkV: " + formatter.format(kV));
                        Logger.recordOutput("TurretKS", kS);
                        Logger.recordOutput("TurretKV", kV);
                    }
                )
        );
    }
}
