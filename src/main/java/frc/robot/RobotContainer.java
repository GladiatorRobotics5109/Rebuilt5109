// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.RobotState.FuelStrategy;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.Visualizer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final DriveSubsystem m_drive;
    private final VisionSubsystem m_vision;
    private final FlywheelsSubsystem m_flywheels;
    private final TurretSubsystem m_turret;
    private final IndexerSubsystem m_indexer;
    private final HoodSubsystem m_hood;
    private final IntakeSubsystem m_intake;

    // Controller
    private final CommandPS4Controller m_driverController = new CommandPS4Controller(Constants.kDriverControllerPort);
    private CommandGenericHID m_driverControllerSim;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        RobotState.init();

        m_drive = new DriveSubsystem();
        m_turret = new TurretSubsystem();
        m_vision = new VisionSubsystem(
            m_drive::addVisionMeasurement,
            m_drive::getRotation,
            () -> Conversions.robotToTurretCamera(m_turret.getPosition())
        );
        m_flywheels = new FlywheelsSubsystem();
        m_indexer = new IndexerSubsystem();
        m_hood = new HoodSubsystem();
        m_intake = new IntakeSubsystem();

        // Set up auto routines
        AutoCommands.init(m_drive, m_turret, m_flywheels, m_indexer);
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
        buildAutoChooser();

        if (Constants.kCurrentMode == Mode.SIM && Constants.kSimShouldUseKeyboard) {
            m_driverControllerSim = new CommandGenericHID(0);
            configureBindingsKeyboard();
        }

        configureBindings();

        if (Constants.kCurrentMode == Mode.SIM) {
            DriverStation.silenceJoystickConnectionWarning(true);

            Visualizer.init(m_drive, m_flywheels);
        }

        CommandScheduler.getInstance().onCommandInitialize(
            command -> Logger.recordOutput("CommandLog", "INIT: " + command.getName())
        );
        CommandScheduler.getInstance().onCommandFinish(
            command -> Logger.recordOutput("CommandLog", "END: " + command.getName())
        );
        CommandScheduler.getInstance().onCommandInterrupt(
            command -> Logger.recordOutput("CommandLog", "INTERRUPT: " + command.getName())
        );
    }

    private void configureBindings() {
        m_drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                () -> m_driverController.getR2Axis(),
                m_drive
            )
        );

        if (Constants.kEnableAutoAimAsDefault) {
            m_flywheels.setDefaultCommand(FlywheelsCommands.autoAim(m_flywheels));
            m_turret.setDefaultCommand(TurretCommands.autoAim(m_turret));
            m_hood.setDefaultCommand(HoodCommands.autoAim(m_hood));
        }

        m_driverController.cross().onTrue(
            Commands.either(IndexerCommands.stop(m_indexer), IndexerCommands.index(m_indexer), m_indexer::isIndexing)
        );
        m_driverController.square().onTrue(IndexerCommands.reverse(m_indexer)).onFalse(IndexerCommands.stop(m_indexer));
        m_driverController.triangle().onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().setFuelStrategy(
                    RobotState.getInstance().getFuelStrategy() == FuelStrategy.HUB
                        ? FuelStrategy.SHUTTLE
                        : FuelStrategy.HUB
                )
            )
        );

        m_driverController.circle().toggleOnTrue(
            Commands.startEnd(
                () -> {
                    m_turret.runPosition(Rotation2d.kZero);
                    m_flywheels.runVelocity(3500);
                },
                () -> {
                    m_turret.stop();
                    m_flywheels.stop();
                },
                m_turret,
                m_flywheels
            )
        );

        // m_driverController.R2().and(() -> (m_driverController.getR2Axis() + 1) / 2.0 > 0.05).whileTrue(
        //     IntakeCommands.testRollers(m_intake, () -> {
        //         double val = Math.pow((m_driverController.getR2Axis() + 1) / 2.0, 2);
        //         Logger.recordOutput("VAL", val);
        //         return 12 * val;
        //     })
        // );

        m_driverController.povUp().whileTrue(IntakeCommands.testPivot(m_intake, () -> -2));
        m_driverController.povDown().whileTrue(IntakeCommands.testPivot(m_intake, () -> 2));

        m_driverController.povRight().onTrue(
            Commands.runOnce(() -> m_flywheels.setVelocityOffset(m_flywheels.getVelocityOffset() + 50))
        );
        m_driverController.povLeft().onTrue(
            Commands.runOnce(() -> m_flywheels.setVelocityOffset(m_flywheels.getVelocityOffset() - 50))
        );

        m_driverController.L1().onTrue(
            Commands.either(
                IntakeCommands.deploy(m_intake),
                IntakeCommands.stow(m_intake),
                () -> m_intake.getState() != IntakeState.DEPLOYED
            )
        );

        // Automatically stow the hood when the robot gets close to the trench so that we don't hit it
        // new Trigger(
        //     () -> {
        //         Pose2d pose = RobotState.getInstance().getPose();
        //         ChassisSpeeds vel = RobotState.getInstance().getVelocityFieldRelative();

        //         final double[] positions = new double[] {
        //             LeftTrench.openingTopLeft.getX(),
        //             LeftTrench.openingTopRight.getX(),
        //             LeftTrench.oppOpeningTopLeft.getX(),
        //             LeftTrench.oppOpeningTopRight.getX(),
        //             RightTrench.openingTopLeft.getX(),
        //             RightTrench.openingTopRight.getX(),
        //             RightTrench.oppOpeningTopLeft.getX(),
        //             RightTrench.oppOpeningTopRight.getX()
        //         };

        //         for (double x : positions) {
        //             double delta = pose.getX() - x;
        //             if (Math.abs(delta) < HoodConstants.kHoodAutoStowThreshold
        //                 && Math.signum(vel.vxMetersPerSecond) == Math.signum(delta))
        //                 return true;
        //         }

        //         return false;
        //     }
        // ).whileTrue(HoodCommands.stow(m_hood));
    }

    private void configureBindingsKeyboard() {
        m_drive.setDefaultCommand(DriveCommands.keyboardDrive(m_drive, m_driverControllerSim.getHID()));

        m_driverControllerSim.button(3).whileTrue(IndexerCommands.index(m_indexer));
    }

    private void buildAutoChooser() {
        m_autoChooser.addOption(
            "Comp_preloadLeft",
            AutoCommands.preloadLeft(m_drive, m_turret, m_flywheels, m_indexer, m_intake)
        );
        m_autoChooser.addOption(
            "Comp_preloadRight",
            AutoCommands.preloadRight(m_drive, m_turret, m_flywheels, m_indexer, m_intake)
        );
        m_autoChooser.addOption(
            "Comp_preloadAndOutpostRight",
            AutoCommands.preloadAndOutpostRight(m_drive, m_turret, m_flywheels, m_indexer, m_intake)
        );

        m_autoChooser.addOption(
            "Comp_preloadAndDepotLeft",
            AutoCommands.preloadAndDepotLeft(m_drive, m_turret, m_flywheels, m_indexer, m_intake)
        );

        m_autoChooser.addOption(
            "Comp_preloadAndCenterLeft",
            AutoCommands.preloadAndCenterLeft(m_drive, m_turret, m_flywheels, m_indexer, m_intake)
        );

        m_autoChooser.addOption(
            "Comp_preloadAndCenterRight",
            AutoCommands.preloadAndCenterRight(m_drive, m_turret, m_flywheels, m_indexer, m_intake)
        );

        // Set up SysId routines
        m_autoChooser.addOption(
            "SysID_DriveWheelRadiusCharacterization",
            DriveCommands.wheelRadiusCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "SysID_DriveSimpleFFCharacterization",
            DriveCommands.feedforwardCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "SysID_Drive (Quasistatic Forward)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "SysID_Drive (Quasistatic Reverse)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        m_autoChooser.addOption(
            "SysID_Drive (Dynamic Forward)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "SysID_Drive (Dynamic Reverse)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        m_autoChooser.addOption("Test", AutoCommands.test(m_drive, m_turret, m_flywheels, m_indexer));
        m_autoChooser.addOption("TestTurret", AutoCommands.testTurret(m_flywheels, m_turret, m_indexer));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { return m_autoChooser.get(); }
}
