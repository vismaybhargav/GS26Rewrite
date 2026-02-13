package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
import frc.robot.TeleopInput;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class DriveFSMSystem extends FSMSystem<DriveFSMSystem.DriveFSMState> {
	public enum DriveFSMState {
		TELEOP
	}

	private CommandSwerveDrivetrain drivetrain;

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
	}

	private void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}
	}
}
