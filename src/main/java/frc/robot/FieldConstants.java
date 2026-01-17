package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.experimental.UtilityClass;

@UtilityClass
public class FieldConstants {
    public double kFieldSizeX = Units.inchesToMeters(651.22);
    public double kFieldSizeY = Units.inchesToMeters(317.69);

    @UtilityClass
    public class Hub {
        public double kHubCenterX = Units.inchesToMeters(182.11);
        public double kHubCenterY = Units.inchesToMeters(158.84);
        public double kHubCenterZ = Units.inchesToMeters(72.0);
        public Translation3d kHubCenterPosition = new Translation3d(kHubCenterX, kHubCenterY, kHubCenterZ);

        public double kOpeningOutsideWidth = Units.inchesToMeters(41.932);

        public Translation2d hubCenterPositionFlipped() { return flip(kHubCenterPosition.toTranslation2d()); }
    }

    public Translation2d flip(Translation2d position) {
        return shouldFlip()
            ? new Translation2d(kFieldSizeX - position.getX(), kFieldSizeY - position.getY())
            : position;
    }

    public boolean shouldFlip() {
        return DriverStation.getAlliance().orElseThrow() == Alliance.Red;
    }
}
