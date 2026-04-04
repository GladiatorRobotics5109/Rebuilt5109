package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.util.Conversions;
import lombok.AllArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

@AllArgsConstructor
public class VisionIOTurretLimelightSim implements VisionIO {
    private final Supplier<Transform3d> m_robotToCamera;

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Transform3d robotToCamera = m_robotToCamera.get();
        Logger.recordOutput(VisionConstants.kLogPath + "/RobotToCamera", robotToCamera);
        Pose3d robotPose = Conversions.toPose3d(RobotState.getInstance().getPose());
        Logger.recordOutput(VisionConstants.kLogPath + "/CameraPose", robotPose.plus(robotToCamera));
    }
}
