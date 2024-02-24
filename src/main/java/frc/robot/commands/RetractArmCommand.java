package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class RetractArmCommand extends Command {
    private final ArmSubsystem arm;

    public RetractArmCommand(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.retract();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        switch (arm.getArmState()) {
            case RETRACTED:
                return true;
        
            default:
                return false;
        }
    }
}
