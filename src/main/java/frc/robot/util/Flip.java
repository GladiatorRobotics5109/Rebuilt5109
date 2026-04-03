package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import lombok.experimental.UtilityClass;

import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

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
        // System.out.println("WAYPOINTS LEN: " + path.getWaypoints().size());
        // for (Waypoint w : path.getWaypoints()) {
        //     System.out.println("Waypoint:" + w.anchor().toString());
        // }
        // List<Waypoint> flippedWaypoints = path.getWaypoints().stream().map(waypoint -> flipY(waypoint)).toList();
        // return new PathPlannerPath(
        //     flippedWaypoints,
        //     path.getGlobalConstraints(),
        //     flipY(path.getIdealStartingState()),
        //     flipY(path.getGoalEndState())
        // );

        PathPlannerPath flipped = PathPlannerPath.fromPathPoints(
            path.getAllPathPoints().stream().map(p -> Flip.flipY(p)).toList(),
            path.getGlobalConstraints(),
            Flip.flipY(path.getGoalEndState())
        );

        return flipped;
    }

    public PathPoint flipY(PathPoint p) {
        if (p.rotationTarget == null && p.constraints == null) {
            return new PathPoint(flipY(p.position));
        }
        else if (p.constraints == null) {
            return new PathPoint(
                flipY(p.position),
                new RotationTarget(p.rotationTarget.position(), flipY(p.rotationTarget.rotation()))
            );
        }
        else {
            return new PathPoint(
                flipY(p.position),
                new RotationTarget(p.rotationTarget.position(), flipY(p.rotationTarget.rotation())),
                p.constraints
            );

        }
    }

    public IdealStartingState flipY(IdealStartingState i) {
        return new IdealStartingState(i.velocityMPS(), flipY(i.rotation()));
    }

    public GoalEndState flipY(GoalEndState e) {
        return new GoalEndState(e.velocityMPS(), flipY(e.rotation()));
    }

    public Pose2d flipY(Pose2d p) {
        return new Pose2d(
            flipY(p.getTranslation()),
            flipY(p.getRotation())
        );
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
