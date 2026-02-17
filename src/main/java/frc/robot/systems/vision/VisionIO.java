package frc.robot.systems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
	@AutoLog
	class VisionIOInputs {
		private boolean connected = false;
		private TargetObservation latestTargetObservation =
			new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
		private PoseObservation[] poseObservations = new PoseObservation[0];
		private int[] tagIds = new int[0];

		/**
		 * Is implementation connected.
		 * @return is implementation connected
		 */
		public boolean isConnected() {
			return connected;
		}

		/**
		 * Set is implementation connected.
		 * @param isConnected is connected.
		 */
		public void setConnected(boolean isConnected) {
			connected = isConnected;
		}

		/**
		 * Get the latest target observation.
		 * @return latest target observation
		 */
		public TargetObservation getLatestTargetObservation() {
			return latestTargetObservation;
		}

		/**
		 * Set the latest target observation.
		 * @param theLatestTargetObservation latest target observation
		 */
		public void setLatestTargetObservation(TargetObservation theLatestTargetObservation) {
			latestTargetObservation = theLatestTargetObservation;
		}

		/**
		 * Get pose observations.
		 * @return pose observations
		 */
		public PoseObservation[] getPoseObservations() {
			return poseObservations;
		}

		/**
		 * Set the pose observations.
		 * @param thePoseObservations pose observations
		 */
		public void setPoseObservations(PoseObservation[] thePoseObservations) {
			poseObservations = thePoseObservations;
		}

		/**
		 * Get the tag ids.
		 * @return tag ids
		 */
		public int[] getTagIds() {
			return tagIds;
		}

		/**
		 * Set the tag ids.
		 * @param theTagIds tag ids.
		 */
		public void setTagIds(int[] theTagIds) {
			tagIds = theTagIds;
		}

	}

	/**
	 * Represents the angle to a simple target, not used for pose estimation.
	 *
	 * @param tx tx
	 * @param ty ty
	 */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
	}

	/**
	 * Represents a robot pose sample used for pose estimation.
	 *
	 * @param timestamp timestamp
	 * @param pose pose
	 * @param ambiguity ambiguitiy
	 * @param tagCount tagCount
	 * @param averageTagDistance avg tag dist
	 * @param type type
	 */
	public static record PoseObservation(
			double timestamp,
			Pose3d pose,
			double ambiguity,
			int tagCount,
			double averageTagDistance,
			PoseObservationType type) {
	}

	enum PoseObservationType {
		MEGATAG_1,
		MEGATAG_2,
		PHOTONVISION
	}

	/**
	 * Update implementation inputs.
	 * @param inputs inputs
	 */
	default void updateInputs(VisionIOInputs inputs) {
	}
}
