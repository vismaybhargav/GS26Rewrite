package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.HardwareMap;
import frc.robot.TeleopInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeFSMSystem extends FSMSystem<IntakeFSMSystem.IntakeFSMState> {
    public enum IntakeFSMState {
        IDLE_EXTENDED,
        IDLE_STOWED,
        IDLE_PARTIAL_EXTENDED,
        MOVING_TO_GOAL,
        INTAKING,
        OUTTAKING
    }

    public enum IntakeGoalPosition {
        STOWED(Degrees.of(0)),
        EXTENDED(Degrees.of(0)),
        PARTIAL(Degrees.of(0));

        private Angle position;

        IntakeGoalPosition(Angle goalPosition) {
            position = goalPosition; 
        }

        public Angle getGoalPosition() {
            return position;
        }
    }

    private IntakeGoalPosition goal;

    // Motion Requests
    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);

    // Motor Objects
    private final TalonFX leaderPivotMotor = new TalonFX(HardwareMap.INTAKE_PIVOT_LEADER_CAN_ID);
    private final TalonFX followerPivotMotor = new TalonFX(HardwareMap.INTAKE_PIVOT_FOLLOWER_CAN_ID);
    private final TalonFX flywheelMotor = new TalonFX(HardwareMap.INTAKE_FLYWHEEL_CAN_ID);


    public IntakeFSMSystem() {
        // ===== PIVOT CONFIGS ====== //
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        // Feedback
        pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_PIVOT_GEARING;

        // Slot0
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kG = IntakeConstants.PIVOT_KG;
        pivotConfig.Slot0.kS = IntakeConstants.PIVOT_KS;
        pivotConfig.Slot0.kV = IntakeConstants.PIVOT_KV;
        pivotConfig.Slot0.kA = IntakeConstants.PIVOT_KA;
        pivotConfig.Slot0.kP = IntakeConstants.PIVOT_KP;
        pivotConfig.Slot0.kI = IntakeConstants.PIVOT_KI;
        pivotConfig.Slot0.kD = IntakeConstants.PIVOT_KD;
        pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        // MotionMagic
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.PIVOT_CRUISE_VELO;
        pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.PIVOT_TARGET_ACCEL;
        pivotConfig.MotionMagic.MotionMagicExpo_kV = IntakeConstants.PIVOT_EXPO_KV;

        // ===== FLYWHEEL CONFIGS ====== //
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        
        // Feedback
        flywheelConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;

        // Slot0
        flywheelConfig.Slot0.kV = IntakeConstants.INTAKE_KV;
        flywheelConfig.Slot0.kA = IntakeConstants.INTAKE_KA;
        flywheelConfig.Slot0.kP = IntakeConstants.INTAKE_KP;
        flywheelConfig.Slot0.kI = IntakeConstants.INTAKE_KI;
        flywheelConfig.Slot0.kD = IntakeConstants.INTAKE_KD;


        leaderPivotMotor.getConfigurator().apply(pivotConfig);
        followerPivotMotor.getConfigurator().apply(pivotConfig);
        flywheelMotor.getConfigurator().apply(flywheelConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				leaderPivotMotor.getPosition(),
				leaderPivotMotor.getVelocity(),
				leaderPivotMotor.getAcceleration(),
				leaderPivotMotor.getMotorVoltage(),
				leaderPivotMotor.getPosition(),
				leaderPivotMotor.getVelocity()
		);

        BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				followerPivotMotor.getPosition(),
				followerPivotMotor.getVelocity(),
				followerPivotMotor.getAcceleration(),
				followerPivotMotor.getMotorVoltage(),
				followerPivotMotor.getPosition(),
				followerPivotMotor.getVelocity()
		);

        BaseStatusSignal.setUpdateFrequencyForAll(
				IntakeConstants.UPDATE_FREQUENCY,
				flywheelMotor.getPosition(),
				flywheelMotor.getVelocity(),
				flywheelMotor.getAcceleration(),
				flywheelMotor.getMotorVoltage(),
				flywheelMotor.getPosition(),
				flywheelMotor.getVelocity()
		);

        leaderPivotMotor.optimizeBusUtilization();
        followerPivotMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();

        reset(); 
    }

    @Override
    public void reset() {
        setCurrentState(IntakeFSMState.IDLE_STOWED);
        goal = IntakeGoalPosition.STOWED;

        update(null);
    }

    @Override
    public void update(TeleopInput input) {
        if (input == null) {
            return;
        }

        Logger.recordOutput("Intake/Current State", getCurrentState());
        Logger.recordOutput("Intake/Pivot Goal", goal);

        switch (getCurrentState()) {
            case IDLE_EXTENDED:
                break;
            case IDLE_STOWED:
                break;
            case IDLE_PARTIAL_EXTENDED: 
                break;
            case INTAKING:
                break;
            case OUTTAKING:
                break;
            case MOVING_TO_GOAL:
                handleMovingToGoalState();
                break;
            default:
                throw new IllegalStateException(
                    "[INTAKE] Cannot update an invalid state: " +
                    getCurrentState().toString()
                );
        }

        setCurrentState(nextState(input));
    }

    @Override
    protected IntakeFSMState nextState(TeleopInput input) {
        if (input == null) {
            return IntakeFSMState.IDLE_STOWED;
        }

        switch (getCurrentState()) {
            case IDLE_EXTENDED:
                if (
                    input.getOuttakeButton() 
                    && !input.getIntakeButton()
                    && !input.getIntakeExtendButton()
                    && !input.getIntakeStowButton()
                    && !input.getIntakePartialButton()
                ) {
                    return IntakeFSMState.OUTTAKING;
                }

                if (
                    input.getIntakeButton() 
                    && !input.getOuttakeButton()
                    && !input.getIntakeExtendButton()
                    && !input.getIntakeStowButton()
                    && !input.getIntakePartialButton()
                ) {
                    return IntakeFSMState.INTAKING;
                }

                if (
                    input.getIntakeStowButton()
                    && !input.getIntakeButton()
                    && !input.getOuttakeButton()
                    && !input.getIntakeExtendButton()
                    && !input.getIntakePartialButton()
                ) {
                    goal = IntakeGoalPosition.STOWED;
                    return IntakeFSMState.MOVING_TO_GOAL;   
                }

                if (
                    input.getIntakePartialButton()
                    && !input.getIntakeButton()
                    && !input.getOuttakeButton()
                    && !input.getIntakeExtendButton()
                    && !input.getIntakeStowButton()
                ) {
                    goal = IntakeGoalPosition.PARTIAL;
                    return IntakeFSMState.MOVING_TO_GOAL;   
                }

                return IntakeFSMState.IDLE_EXTENDED;
            case IDLE_STOWED:
                if (
                    input.getIntakePartialButton()
                    && !input.getIntakeButton()
                    && !input.getOuttakeButton()
                    && !input.getIntakeExtendButton()
                    && !input.getIntakeStowButton()
                ) {
                    goal = IntakeGoalPosition.PARTIAL;
                    return IntakeFSMState.MOVING_TO_GOAL;   
                }

                if (
                    input.getIntakeExtendButton()
                    && !input.getIntakeButton()
                    && !input.getOuttakeButton()
                    && !input.getIntakeStowButton()
                    && !input.getIntakePartialButton()
                ) {
                    goal = IntakeGoalPosition.EXTENDED;
                    return IntakeFSMState.MOVING_TO_GOAL;   
                }

                return IntakeFSMState.IDLE_STOWED;
            case IDLE_PARTIAL_EXTENDED:
                if (
                    input.getIntakeExtendButton()
                    && !input.getIntakeButton()
                    && !input.getOuttakeButton()
                    && !input.getIntakeStowButton()
                    && !input.getIntakePartialButton()
                ) {
                    goal = IntakeGoalPosition.EXTENDED;
                    return IntakeFSMState.MOVING_TO_GOAL;   
                }

                if (
                    input.getIntakeStowButton()
                    && !input.getIntakeButton()
                    && !input.getOuttakeButton()
                    && !input.getIntakeExtendButton()
                    && !input.getIntakePartialButton()
                ) {
                    goal = IntakeGoalPosition.STOWED;
                    return IntakeFSMState.MOVING_TO_GOAL;   
                }

                return IntakeFSMState.IDLE_PARTIAL_EXTENDED;
            case INTAKING:
                if (
                    !input.getIntakeButton()
                ) {
                    return IntakeFSMState.IDLE_EXTENDED;
                }

                return IntakeFSMState.INTAKING;
            case OUTTAKING:
                if (
                    !input.getOuttakeButton()
                ) {
                    return IntakeFSMState.IDLE_EXTENDED;
                }

                return IntakeFSMState.OUTTAKING;
            case MOVING_TO_GOAL:
                if (atGoal(goal)) {
                    switch (goal) {
                        case EXTENDED:
                            return IntakeFSMState.IDLE_EXTENDED;
                        case STOWED:
                            return IntakeFSMState.IDLE_STOWED;
                        case PARTIAL:
                            return IntakeFSMState.IDLE_PARTIAL_EXTENDED;
                    }
                }
                return IntakeFSMState.MOVING_TO_GOAL;
            default:
                throw new IllegalStateException(
                    "[INTAKE] Cannot get next state of an invalid state: " +
                    getCurrentState().toString()
                );
        }
    }

    public void handleMovingToGoalState() {
        leaderPivotMotor.setControl(pivotPositionRequest.withPosition(goal.getGoalPosition()));
    }

    public boolean atGoal(IntakeGoalPosition position) {
        return true;
    }
}
