// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.LeftTrench;
import frc.robot.FieldConstants.RightTrench;
import frc.robot.RobotState.FuelStrategy;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.flywheels.FlywheelsSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.Visualizer;
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
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
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
    }

    private void configureBindings() {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()
            )
        );

        m_flywheels.setDefaultCommand(FlywheelsCommands.autoAim(m_flywheels));
        m_turret.setDefaultCommand(TurretCommands.autoAim(m_turret));

        m_driverController.circle().whileTrue(IndexerCommands.index(m_indexer));
        m_driverController.triangle().onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().setFuelStrategy(
                    RobotState.getInstance().getFuelStrategy() == FuelStrategy.HUB
                        ? FuelStrategy.SHUTTLE_AUTO
                        : FuelStrategy.HUB
                )
            )
        );

        // Automatically stow the hood when the robot gets close to the trench so that we don't hit it
        new Trigger(
            () -> {
                Pose2d pose = RobotState.getInstance().getPose();
                ChassisSpeeds vel = RobotState.getInstance().getVelocityFieldRelative();

                final double[] positions = new double[] {
                    LeftTrench.openingTopLeft.getX(),
                    LeftTrench.openingTopRight.getX(),
                    LeftTrench.oppOpeningTopLeft.getX(),
                    LeftTrench.oppOpeningTopRight.getX(),
                    RightTrench.openingTopLeft.getX(),
                    RightTrench.openingTopRight.getX(),
                    RightTrench.oppOpeningTopLeft.getX(),
                    RightTrench.oppOpeningTopRight.getX()
                };

                for (double x : positions) {
                    double delta = pose.getX() - x;
                    if (Math.abs(delta) < HoodConstants.kHoodAutoStowThreshold
                        && Math.signum(vel.vxMetersPerSecond) == Math.signum(delta))
                        return true;
                }

                return false;
            }
        ).whileTrue(HoodCommands.stow(m_hood));
    }

    private void configureBindingsKeyboard() {
        m_drive.setDefaultCommand(DriveCommands.keyboardDrive(m_drive, m_driverControllerSim.getHID()));

        m_driverControllerSim.button(3).whileTrue(IndexerCommands.index(m_indexer));
    }

    private void buildAutoChooser() {
        // Set up SysId routines
        m_autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        m_autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { return m_autoChooser.get(); }
}
