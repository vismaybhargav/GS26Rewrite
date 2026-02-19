package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL);

	// Intake CAN IDS
	public static final int INTAKE_PIVOT_LEADER_CAN_ID = 11;
	public static final int INTAKE_PIVOT_FOLLOWER_CAN_ID = 12;
	public static final int INTAKE_FLYWHEEL_CAN_ID = 5;

	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}

	/**
	 * Hardware map entry for the example FSM.
	 * @return if the hardware for the example FSM is present
	 */
	public static boolean isDrivetrainEnabled() {
		return true;
	}
}
