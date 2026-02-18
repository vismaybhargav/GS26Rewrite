package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import frc.robot.TeleopInput;

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
        STOWED(0.0),
        EXTENDED(0.0),
        PARTIAL(0.0);

        private double position;

        IntakeGoalPosition(double goalPosition) {
            position = goalPosition; 
        }

        public double getGoalPosition() {
            return position;
        }
    }

    private IntakeGoalPosition goal;

    public IntakeFSMSystem() {
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
        Logger.recordOutput("Intake/Goal", goal);
        Logger.recordOutput("Intake/Inputs/Partial", input.getIntakePartialButton());
        Logger.recordOutput("Intake/Inputs/Extend", input.getIntakeExtendButton());
        Logger.recordOutput("Intake/Inputs/Stow", input.getIntakeStowButton());
        Logger.recordOutput("Intake/Inputs/Outtake", input.getOuttakeButton());
        Logger.recordOutput("Intake/Inputs/Intake", input.getIntakeButton());


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

    public boolean atGoal(IntakeGoalPosition position) {
        return true;
    }
}
