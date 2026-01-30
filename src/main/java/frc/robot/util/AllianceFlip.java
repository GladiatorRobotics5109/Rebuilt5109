package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import lombok.experimental.UtilityClass;

import java.util.Optional;

@UtilityClass
public class AllianceFlip {
    public boolean shouldFlip() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            DriverStation.reportWarning("Couldn't get alliance (AllianceFlip::shouldFlip)", true);

            return false;
        }

        return alliance.get() == Alliance.Red;
    }

    public Translation3d apply(Translation3d t) {
        return shouldFlip()
            ? new Translation3d(
                FieldConstants.fieldLength - t.getX(),
                FieldConstants.fieldWidth - t.getY(),
                t.getZ()
            )
            : t;
    }

    public double flipX(double x) {
        return FieldConstants.fieldLength - x;
    }
}
