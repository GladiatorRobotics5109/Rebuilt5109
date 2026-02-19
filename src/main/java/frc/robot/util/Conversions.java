package frc.robot.util;

import edu.wpi.first.math.geometry.*;
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

    public Pose3d toPose3d(Pose2d pose2d) {
        return new Pose3d(
            pose2d.getX(),
            pose2d.getY(),
            0.0,
            new Rotation3d(pose2d.getRotation())
        );
    }
}
