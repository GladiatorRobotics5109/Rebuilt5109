package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import lombok.experimental.UtilityClass;

@UtilityClass
public class Conversions {
    public Transform3d robotToTurretCamera(Rotation2d turretAngle) {
        Rotation3d turretAngle3d = new Rotation3d(turretAngle);

        Transform3d turretToCamera = new Transform3d(
            VisionConstants.kTurretToCamera1.getTranslation().rotateBy(turretAngle3d),
            VisionConstants.kTurretToCamera1.getRotation().plus(turretAngle3d)
        );

        return TurretConstants.kRobotToTurret.plus(turretToCamera);
    }

    public Pose3d robotPoseToTurretPose(Pose2d robot, Rotation2d turretAngle, Rotation2d hoodAngle) {
        Translation3d robotTranslation = toTranslation3d(robot.getTranslation());
        Translation3d translation = robotTranslation.plus(
            TurretConstants.kRobotToTurret.getTranslation().rotateBy(new Rotation3d(robot.getRotation()))
        );

        return new Pose3d(
            translation,
            new Rotation3d(0.0, hoodAngle.getRadians(), robot.getRotation().getRadians() + turretAngle.getRadians())
        );
    }

    public Pose3d toPose3d(Pose2d pose2d) {
        return new Pose3d(
            pose2d.getX(),
            pose2d.getY(),
            0.0,
            new Rotation3d(pose2d.getRotation())
        );
    }

    public Translation3d toTranslation3d(Translation2d translation2d) {
        return new Translation3d(
            translation2d.getX(),
            translation2d.getY(),
            0.0
        );
    }

    public double radiansToRotations(double rad) {
        return Units.radiansToRotations(rad);
    }

    public double inchesToMeters(double in) {
        return Units.inchesToMeters(in);
    }

    public double degreesToRadians(double deg) {
        return Units.degreesToRadians(deg);
    }
}
