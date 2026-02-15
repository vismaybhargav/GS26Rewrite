package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
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

		double xSpeed = MathUtil.applyDeadband(
			-input.getDriverLeftX(),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND
		) * TunerConstants.SPEED_12V.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
			-input.getDriverLeftY(),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND
		) * TunerConstants.SPEED_12V.in(MetersPerSecond);

		double aSpeed = MathUtil.applyDeadband(
			-input.getDriverLeftY(),
			DrivetrainConstants.TRANSLATIONAL_DEADBAND
		) * DrivetrainConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

		drivetrain.setControl(
			driveFieldCentric
				.withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
				.withRotationalRate(aSpeed * DrivetrainConstants.ROTATIONAL_DAMP)
		);
	}
}
