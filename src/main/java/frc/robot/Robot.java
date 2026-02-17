// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_LIMELIGHT;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import frc.robot.Constants.VisionConstants;

// WPILib Imports

// Systems
import frc.robot.motors.MotorManager;
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.vision.Vision;
import frc.robot.systems.vision.VisionIOLimelight;
import frc.robot.systems.vision.VisionIOPhotonVisionSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	private Vision vision;
	private DriveFSMSystem driveFSMSystem;

	// Systems

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		driveFSMSystem = new DriveFSMSystem();

		if (isReal()) {
			vision = new Vision(
				driveFSMSystem::addVisionMeasurement,
				new VisionIOLimelight(
					VisionConstants.LIMELIGHT_NAME, 
					() -> driveFSMSystem.getPose().getRotation()
				)
			);
		} else if (isSimulation()) {
			vision = new Vision(
				driveFSMSystem::addVisionMeasurement,
				new VisionIOPhotonVisionSim(
					VisionConstants.LIMELIGHT_NAME,
					ROBOT_TO_LIMELIGHT,
					() -> driveFSMSystem.getPose()
				)
			);
		}

		Logger.recordMetadata("GS26Rewrite", "Robot Code");
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
	}

	@Override
	public void autonomousPeriodic() {
		// logs motor values
		MotorManager.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveFSMSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);

		// logs motor values
		MotorManager.update();
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() {

	}

	@Override
	public void robotPeriodic() { 
		vision.periodic();
	}
}
