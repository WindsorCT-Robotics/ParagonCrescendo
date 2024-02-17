package frc.robot.commands;

import frc.robot.Constants.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class HomeArmCommand extends Command {

    private final ArmSubsystem arm;

    public HomeArmCommand(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.moveArm(-Arm.ROTATION_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        if (arm.isHome()) {
            arm.setInsideArmState();
            return true;
        }
        return false;
    }
}
