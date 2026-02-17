package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.TeleopInput;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class DriveFSMSystem extends FSMSystem<DriveFSMSystem.DriveFSMState> {
	public enum DriveFSMState {
		TELEOP
	}

	private CommandSwerveDrivetrain drivetrain;

	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
		.withDeadband(
			TunerConstants.SPEED_12V.in(MetersPerSecond)
			* DrivetrainConstants.TRANSLATIONAL_DEADBAND
		)
		.withRotationalDeadband(
			DrivetrainConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
			* DrivetrainConstants.ROTATIONAL_DEADBAND
		)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	/**
	 * The driveFSMSystem.
	 */
	public DriveFSMSystem() {
		drivetrain = TunerConstants.createDrivetrain();

		reset();
	}

	@Override
	public void reset() {
		setCurrentState(DriveFSMState.TELEOP);

		update(null);
	}

	@Override
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}

		drivetrain.periodic();

		switch (getCurrentState()) {
			case TELEOP:
				handleTeleopState(input);
				break;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot update an invalid state: " + getCurrentState().toString()
				);
		}
	}

	@Override
	protected DriveFSMState nextState(TeleopInput input) {
		if (input == null) {
			return DriveFSMState.TELEOP;
		}

		switch (getCurrentState()) {
			case TELEOP:
				return DriveFSMState.TELEOP;
			default:
				throw new IllegalStateException(
					"[DRIVETRAIN] Cannot get next state of an invlid state: " +
					getCurrentState().toString()
				);
		}
	}

	private void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}

		double ySpeed = MathUtil.applyDeadband(
			input.getDriverLeftX(),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND
		) * TunerConstants.SPEED_12V.in(MetersPerSecond);

		double xSpeed = MathUtil.applyDeadband(
			input.getDriverLeftY(),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND
		) * TunerConstants.SPEED_12V.in(MetersPerSecond);

		double aSpeed = MathUtil.applyDeadband(
			-input.getDriverRightX(),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND
		) * DrivetrainConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

		drivetrain.setControl(
			driveFieldCentric
				.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withRotationalRate(aSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);
	}

	/**
	 * Get the underlying cmd swerve drivetrain
	 * @return cmd swerve drivetrain
	 */
	public CommandSwerveDrivetrain getCommandSwerveDrivetrain() {
		return drivetrain;
	}

	/**
	 * Get the current drivetrain pose
	 * @return current drivetrain pose
	 */
	@AutoLogOutput(key = "Drivetrain/Pose")
	public Pose2d getPose() {
		return drivetrain.getState().Pose;
	}

	/**
	 * Get the curr chassis speeds
	 * @return chassis speeds
	 */
	@AutoLogOutput(key = "Drivetrain/Chassis Speeds")
	public ChassisSpeeds getChassisSpeeds() {
		return drivetrain.getState().Speeds;
	}

	/**
	 * Get the drivetrain states.
	 *
	 * @return the swerve module states
	 */
	@AutoLogOutput(key = "Drivetrain/Measured States")
	public SwerveModuleState[] getModuleStates() {
		return drivetrain.getState().ModuleStates;
	}

	/**
	 * Get the drivetrain targets.
	 *
	 * @return drivetrain targets
	 */
	@AutoLogOutput(key = "Drivetrain/Target States")
	public SwerveModuleState[] getModuleTargets() {
		return drivetrain.getState().ModuleTargets;
	}

	/**
	 * Get the drivetrain positions.
	 *
	 * @return drivetrain targets
	 */
	@AutoLogOutput(key = "Drivetrain/Positions")
	public SwerveModulePosition[] getModulePositions() {
		return drivetrain.getState().ModulePositions;
	}

	/**
	 * Adds a new timestamped vision measurement.
	 *
	 * @param visionPoseMeters The pose of the robot in the camera's coordinate
	 *                         frame
	 * @param timestampSeconds The timestamp of the measurement
	 * @param visionStdDevs    The standard deviations of the measurement in the x,
	 *                         y, and theta directions
	 */
	public void addVisionMeasurement(
			Pose2d visionPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionStdDevs) {
		drivetrain.addVisionMeasurement(
				visionPoseMeters,
				timestampSeconds,
				visionStdDevs);
	}
}
