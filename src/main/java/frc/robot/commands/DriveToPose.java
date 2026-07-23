package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.DriveConstants.*;

public class DriveToPose extends Command {
    private final DriveSubsystem m_drive;

    private final ProfiledPIDController m_angle = new ProfiledPIDController(
        kDriveToPoseAngleP,
        0.0,
        kDriveToPoseAngleD,
        new Constraints(kDriveToPoseAngleMaxVelocity, kDriveToPoseAngleMaxAcceleration)
    );
    private final ProfiledPIDController m_x = new ProfiledPIDController(
        kDriveToPoseXP,
        0.0,
        kDriveToPoseXD,
        new Constraints(kDriveToPoseXMaxVelocity, kDriveToPoseXMaxAcceleration)
    );
    private final ProfiledPIDController m_y = new ProfiledPIDController(
        kDriveToPoseYP,
        0.0,
        kDriveToPoseYD,
        new Constraints(kDriveToPoseYMaxVelocity, kDriveToPoseYMaxAcceleration)
    );

    private final Supplier<Pose2d> m_targetPoseSupplier;
    private Pose2d m_targetPose;

    public DriveToPose(Supplier<Pose2d> targetPose, DriveSubsystem drive) {
        addRequirements(drive);
        setName("DriveToPose");

        m_angle.enableContinuousInput(-Math.PI, Math.PI);

        m_targetPoseSupplier = targetPose;

        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_targetPose = m_targetPoseSupplier.get();

        Pose2d current = m_drive.getPose();

        m_x.reset(current.getX());
        m_y.reset(current.getY());
        m_angle.reset(current.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d current = m_drive.getPose();

        double x = m_x.calculate(current.getX(), m_targetPose.getX());
        double y = m_y.calculate(current.getY(), m_targetPose.getY());

        double omega = m_angle.calculate(
            current.getRotation().getRadians(),
            m_targetPose.getRotation().getRadians()
        );

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, current.getRotation());

        m_drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        Pose2d current = m_drive.getPose();
        return MathUtil.isNear(m_targetPose.getX(), current.getX(), kDriveToPoseToleranceX)
            && MathUtil.isNear(m_targetPose.getY(), current.getY(), kDriveToPoseToleranceY)
            && MathUtil.isNear(
                m_targetPose.getRotation().getRadians(),
                current.getRotation().getRadians(),
                kDriveToPoseToleranceAngle
            );
    }
}
