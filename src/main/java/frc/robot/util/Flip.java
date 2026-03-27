package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import lombok.experimental.UtilityClass;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

@UtilityClass
public class Flip {
    public boolean shouldFlip() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            DriverStation.reportWarning("Couldn't get alliance (AllianceFlip::shouldFlip)", true);

            return false;
        }

        return alliance.get() == Alliance.Red;
    }

    public Pose2d apply(Pose2d p) {
        return shouldFlip() ? flip(p) : p;
    }

    public Translation3d apply(Translation3d t) {
        return shouldFlip() ? flip(t) : t;
    }

    public Translation2d apply(Translation2d t) {
        return shouldFlip() ? flip(t) : t;
    }

    public double applyX(double x) {
        return shouldFlip() ? flipX(x) : x;
    }

    public double flipX(double x) {
        return FieldConstants.fieldLength - x;
    }

    public Pose2d flip(Pose2d p) {
        return new Pose2d(
            FieldConstants.fieldLength - p.getX(),
            FieldConstants.fieldWidth - p.getY(),
            p.getRotation().plus(Rotation2d.k180deg)
        );
    }

    public Translation3d flip(Translation3d t) {
        return new Translation3d(
            FieldConstants.fieldLength - t.getX(),
            FieldConstants.fieldWidth - t.getY(),
            t.getZ()
        );
    }

    public Translation2d flip(Translation2d t) {
        return new Translation2d(FieldConstants.fieldLength - t.getX(), FieldConstants.fieldWidth - t.getY());
    }

    public PathPlannerPath flipY(PathPlannerPath path) {
        List<Waypoint> flippedWaypoints = path.getWaypoints().stream().map(waypoint -> flipY(waypoint)).toList();
        return new PathPlannerPath(
            flippedWaypoints,
            path.getGlobalConstraints(),
            flipY(path.getIdealStartingState()),
            flipY(path.getGoalEndState())
        );
    }

    public Waypoint flipY(Waypoint w) {
        return new Waypoint(flipY(w.prevControl()), flipY(w.anchor()), flipY(w.nextControl()));
    }

    public IdealStartingState flipY(IdealStartingState i) {
        return new IdealStartingState(i.velocityMPS(), flipY(i.rotation()));
    }

    public GoalEndState flipY(GoalEndState e) {
        return new GoalEndState(e.velocityMPS(), flipY(e.rotation()));
    }

    public Rotation2d flipY(Rotation2d r) {
        return r.times(-1);
    }

    public Translation2d flipY(Translation2d t) {
        return new Translation2d(
            t.getX(),
            FieldConstants.fieldWidth - t.getY()
        );
    }
}
