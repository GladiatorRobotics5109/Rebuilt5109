package frc.robot.subsystems.turret;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants.TurretConstants;
import lombok.AllArgsConstructor;
import lombok.Getter;
import org.opencv.core.Mat;

import java.util.NavigableMap;
import java.util.TreeMap;

public class TurretPoseEstimator {
    private final Translation3d m_robotToTurret;

    @Getter
    private Pose3d m_pose;
    @Getter
    private ChassisSpeeds m_velocity;
    
    // Inputs:
    // 0-3 = (x, y, z) position of turret field relative
    // 4-6 = (v_x, v_y, v_z) velocity of turret field relative
    // 7 = (omega) angular velocity of turret
    private final KalmanFilter<N7, N0, N7> m_kalman;
    private KalmanFilterLatencyCompensator<N2, N1, N2> m_latencyCompensator;
    
    public TurretPoseEstimator(Translation3d robotToTurret, Pose3d initialPose, ChassisSpeeds initialVelocity, double turretKV, double turretKA) {
        m_robotToTurret = robotToTurret;
        
        m_pose = initialPose;
        m_velocity = initialVelocity;
        
        // Linear velocity dampening
        double k1 = 0.8;
        // Angular velocity dampening
        double k2 = 0.8;
        
        Matrix<N7, N7> A = MatBuilder.fill(Nat.N7(), Nat.N7(),
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, -k1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, -k1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -k1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -k2
        );
        
        Matrix<N7, N0> B = MatBuilder.fill(Nat.N7(), Nat.N0());
        
        Matrix<N7, N7> C = Matrix.eye(Nat.N7());
        // MatBuilder.fill(Nat.N7(), Nat.N7(),
        //     1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        //     0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        //     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        //     0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        //     0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        //     0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        // );
        
        Matrix<N7, N0> D = MatBuilder.fill(Nat.N7(), Nat.N0());
        
        m_kalman = new KalmanFilter<>(
            Nat.N7(),
            Nat.N7(),
            new LinearSystem<>(A, B, C, D),
            MatBuilder.fill(Nat.N7(), Nat.N1(), 0.0, 0.0),
            MatBuilder.fill(Nat.N7(), Nat.N1(), 0.0, 0.0),
            0.02
        );
    }

    public void update(
        Pose2d robotPose,
        ChassisSpeeds robotVelocity,
        Rotation2d turretAngle,
        Rotation2d turretVelocity
    ) {
        Rotation3d robotRotation = new Rotation3d(robotPose.getRotation());

        Pose3d estimatedPose = new Pose3d(robotPose.getX(), robotPose.getY(), 0.0, robotRotation)
            .plus(new Transform3d(m_robotToTurret.rotateBy(robotRotation), Rotation3d.kZero))
            .plus(new Transform3d(Translation3d.kZero, new Rotation3d(turretAngle)));

        double radius = m_robotToTurret.toTranslation2d().getNorm();
        Rotation2d angleFieldRelative = m_robotToTurret.toTranslation2d().getAngle().plus(robotPose.getRotation());
        double tangentialVelocity = robotVelocity.omegaRadiansPerSecond * radius;
        Rotation2d tangentialVelocityAngle = angleFieldRelative.plus(Rotation2d.kCCW_Pi_2);
        double tangentialVelocityX = tangentialVelocity * tangentialVelocityAngle.getCos();
        double tangentialVelocityY = tangentialVelocity * tangentialVelocityAngle.getSin();

        ChassisSpeeds estimatedVelocity = new ChassisSpeeds(
            robotVelocity.vxMetersPerSecond + tangentialVelocityX,
            robotVelocity.vyMetersPerSecond + tangentialVelocityY,
            turretVelocity.getRadians()
        );
        
        m_kalman.correct(
            MatBuilder.fill(Nat.N1(), Nat.N1(), 0.0),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0, 0.0, 0.0)
        );
    }

    public void addVisionMeasurement(Pose3d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    
    }
}
