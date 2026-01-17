package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.vision.Vision;

public class RobotState {
    private static Drive s_drive;
    private static Vision s_vision;
    private static Flywheels s_flywheels;

    public static void initDrive(Drive drive) { s_drive = drive; }

    public static void initVision(Vision vision) { s_vision = vision; }

    public static void initFlywheels(Flywheels flywheels) { s_flywheels = flywheels; }

    public static void verifyInit() {
        if (s_drive == null) {
            DriverStation.reportWarning("Drive state has not been initialized!", true);
        }
        if (s_vision == null) {
            DriverStation.reportWarning("Vision state has not been initialized!", true);
        }
        if (s_flywheels == null) {
            DriverStation.reportWarning("Flywheels state has not been initialized!", true);
        }
    }

    public static Pose2d getPose() { return s_drive.getPose(); }

    public static Rotation2d getRotation() { return s_drive.getRotation(); }
}
