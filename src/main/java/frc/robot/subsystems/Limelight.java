package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private Optional<Measurement> lastMeasurement = Optional.empty();

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (poseEstimate_MegaTag1 == null
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0) {
            lastMeasurement = Optional.empty();
            return Optional.empty();
        }

        // Combine MT2 translation with MT1 rotation
        poseEstimate_MegaTag2.pose = new Pose2d(
                poseEstimate_MegaTag2.pose.getTranslation(),
                poseEstimate_MegaTag1.pose.getRotation());

        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);
        final Measurement measurement = new Measurement(poseEstimate_MegaTag2, standardDeviations);

        lastMeasurement = Optional.of(measurement);
        posePublisher.set(poseEstimate_MegaTag2.pose);

        return lastMeasurement;
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }

    public boolean hasValidMeasurement() {
        return lastMeasurement.isPresent();
    }

    public Optional<Pose2d> getLastEstimatedPose() {
        return lastMeasurement.map(m -> m.poseEstimate.pose);
    }

    public long getLastTagCount() {
        return lastMeasurement.map(m -> (long) m.poseEstimate.tagCount).orElse(0L);
    }

    public double getLastTimestampSeconds() {
        return lastMeasurement.map(m -> m.poseEstimate.timestampSeconds).orElse(0.0);
    }

    public double getLastStdDevX() {
        return lastMeasurement.map(m -> m.standardDeviations.get(0, 0)).orElse(0.0);
    }

    public double getLastStdDevY() {
        return lastMeasurement.map(m -> m.standardDeviations.get(1, 0)).orElse(0.0);
    }

    public double getLastStdDevTheta() {
        return lastMeasurement.map(m -> m.standardDeviations.get(2, 0)).orElse(0.0);
    }
}
