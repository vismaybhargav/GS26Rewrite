package frc.robot.systems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.VisionIOInputsAutoLogged;
import frc.robot.systems.vision.VisionIO.PoseObservationType;

public class Vision {
	private final VisionConsumer consumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;

	/**
	 * Constructor for the Vision subsystem.
	 * @param visionConsumer the drivetrain basically
	 * @param iO implementations
	 */
	public Vision(VisionConsumer visionConsumer, VisionIO... iO) {
		consumer = visionConsumer;
		io = iO;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert(
					"Vision camera " + Integer.toString(i) + " is disconnected.",
						AlertType.kWarning);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing
	 * with vision.
	 *
	 * @param cameraIndex The index of the camera to use.
	 * @return target vector
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].getLatestTargetObservation().tx();
	}

	/**
	 * Periodic func.
	 */
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isConnected());

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].getTagIds()) {
				var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].getPoseObservations()) {
				// Check whether to reject pose
				boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
						|| (observation.tagCount() == 1
								&& observation.ambiguity()
								> VisionConstants.MAX_AMBIGUITY) // Cannot be high ambiguity
						|| Math.abs(observation.pose().getZ())
							> VisionConstants.MAX_Z_ERROR // Must have realistic Z coordinate

						// Must be within the field boundaries
						|| observation.pose().getX() < 0.0
						|| observation.pose().getX()
							> VisionConstants.TAG_LAYOUT.getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY()
							> VisionConstants.TAG_LAYOUT.getFieldWidth();

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double stdDevFactor = Math.pow(
					observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = VisionConstants.LINEAR_STD_DEV_BASELINE * stdDevFactor;
				double angularStdDev = VisionConstants.ANGULAR_STD_DEV_BASELINE * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= VisionConstants.LINEAR_STD_DEV_MT2_FACTOR;
					angularStdDev *= VisionConstants.ANGULAR_STD_DEV_MT2_FACTOR;
				}
				if (cameraIndex < VisionConstants.CAMERA_STD_DEV_FACTORS.length) {
					linearStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
					angularStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
				}

				// Send vision observation
				consumer.accept(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			// Log camera metadata
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[0]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[0]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[0]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[0]));
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
		Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
	}

	@FunctionalInterface
	public interface VisionConsumer {
		/**
		 * Accepts a vision observation.
		 * @param visionRobotPoseMeters vision robot pose
		 * @param timestampSeconds timestamp of observation
		 * @param visionMeasurementStdDevs standard devations
		 */
		void accept(
				Pose2d visionRobotPoseMeters,
				double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
