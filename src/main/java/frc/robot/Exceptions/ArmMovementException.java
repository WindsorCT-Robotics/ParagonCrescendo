package frc.robot.Exceptions;

import frc.robot.subsystems.ArmSubsystem.ArmState;

public class ArmMovementException extends RuntimeException {
    public ArmMovementException(ArmState currentState, ArmState nextState) {
        super("Unable to transition arm to state " + nextState.toString() + " from current state " + currentState.toString() + ".");
    }
}