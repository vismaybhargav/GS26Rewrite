package frc.robot.systems.vision;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;

public class VisionIOLimelight implements VisionIO {
    private final Limelight limelight;

    private final LimelightPoseEstimator mt1PoseEstimator;
    private final LimelightPoseEstimator mt2PoseEstimator;

    public VisionIOLimelight(String limelightName, Supplier<Rotation2d> rotationSupplier) {
        limelight = new Limelight(limelightName);

        limelight
            .getSettings()
            .withImuMode(ImuMode.InternalImuExternalAssist)
            .withCameraOffset(
                new Pose3d(
                    VisionConstants.ROBOT_TO_LIMELIGHT.getTranslation(), 
                    VisionConstants.ROBOT_TO_LIMELIGHT.getRotation()
                )
            )
            .save();

        mt1PoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
        mt2PoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.setConnected(true);

        var limelightData = limelight.getData();
        var poseObservations = new PoseObservation[2];
        var ambiguity = limelightData.getRawFiducials()[0].ambiguity;
        var targetData = limelightData.targetData;

        mt1PoseEstimator.getPoseEstimate().ifPresent(poseEstimate -> {
            poseObservations[0] = new PoseObservation(
                poseEstimate.timestampSeconds,
                poseEstimate.pose,
                ambiguity,
                poseEstimate.tagCount,
                poseEstimate.avgTagDist,
                PoseObservationType.MEGATAG_1
            );
        });

        mt2PoseEstimator.getPoseEstimate().ifPresent(poseEstimate -> {
            poseObservations[1] = new PoseObservation(
                poseEstimate.timestampSeconds,
                poseEstimate.pose,
                ambiguity,
                poseEstimate.tagCount,
                poseEstimate.avgTagDist,
                PoseObservationType.MEGATAG_2
            );
        });

        inputs.setLatestTargetObservation(
            new TargetObservation(
                new Rotation2d(targetData.getHorizontalOffset(), 0), 
                new Rotation2d(0, targetData.getVerticalOffset())
            )
        );

        inputs.setPoseObservations(poseObservations); 
        inputs.setTagIds(Arrays.stream(limelightData.getRawFiducials()).mapToInt(fid -> fid.id).toArray());
    }
}
