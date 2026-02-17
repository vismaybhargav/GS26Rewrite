package frc.robot.systems.vision;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVisionSim implements VisionIO {
	private final PhotonCamera camera;
	private final PhotonPoseEstimator poseEstimator;
	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;
	private final VisionSystemSim visionSystemSim;


	/**
	 * Creates a new VisionIOPhotonPoseEstimator.
	 * @param name The name of the camera.
	 * @param roboToCamera The transform from the camera to the robot.
	 * @param supplier pose supplier
	 */
	public VisionIOPhotonVisionSim(
		String name,
		Transform3d roboToCamera,
		Supplier<Pose2d> supplier
	) {
		camera = new PhotonCamera(name);
		poseEstimator = new PhotonPoseEstimator(TAG_LAYOUT, roboToCamera);

		poseSupplier = supplier;

		visionSystemSim = new VisionSystemSim("main");
		visionSystemSim.addAprilTags(TAG_LAYOUT);

		var camProps = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(camera, camProps, TAG_LAYOUT);
		visionSystemSim.addCamera(cameraSim, roboToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSystemSim.update(poseSupplier.get());
		inputs.setConnected(camera.isConnected());

		Optional<EstimatedRobotPose> estPose = Optional.empty();
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();
		for (var result : camera.getAllUnreadResults()) {
			estPose = poseEstimator.estimateCoprocMultiTagPose(result);

			if (estPose.isPresent()) {
				var pose = estPose.get();

				double totalTagDistance = 0;
				for (var target : pose.targetsUsed) {
					totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
					tagIds.add((short) target.getFiducialId());
				}
				double averageTagDistance = totalTagDistance / pose.targetsUsed.size();

				poseObservations.add(
					new PoseObservation(
						result.getTimestampSeconds(),
						estPose.get().estimatedPose,
						estPose.get().targetsUsed.get(0).poseAmbiguity,
						estPose.get().targetsUsed.size(),
						averageTagDistance,
						PoseObservationType.PHOTONVISION
					)
				);
			}
		}

		inputs.setPoseObservations(new PoseObservation[poseObservations.size()]);
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.getPoseObservations()[i] = poseObservations.get(i);
		}

		inputs.setTagIds(new int[tagIds.size()]);
		int i = 0;
		for (int id : tagIds) {
			inputs.getTagIds()[i++]  = id;
		}
	}

	/**
	 * Returns the camera.
	 * @return The camera.
	 */
	public PhotonCamera getCamera() {
		return camera;
	}

	/**
	 * Returns the pose estimator.
	 * @return The pose estimator
	 */
	public PhotonPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}
}
