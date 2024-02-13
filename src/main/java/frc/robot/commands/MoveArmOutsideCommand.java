package frc.robot.commands;
import frc.robot.Constants.*;
import frc.robot.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class MoveArmOutsideCommand extends Command {
    private final ArmSubsystem arm;
    private Rotations goalPosition;

    public MoveArmOutsideCommand(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        goalPosition = new Rotations(arm.getEncoderPosition() + Arm.ROTATION_CAP);
    }

    @Override
    public void execute() {
        arm.moveArm(Arm.ROTATION_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        if (!arm.getArmState().equals(ArmState.OUTSIDE)) {
            if (arm.getEncoderPosition() >= goalPosition.asDouble()) {
                arm.setOutsideArmState();
                return true;
            }
        }
        return false;
    }
}
