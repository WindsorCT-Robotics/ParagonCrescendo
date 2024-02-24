package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

public class OuttakeRollersAdjustCommand extends Command {
    private final OuttakeSubsystem outtake;

    public OuttakeRollersAdjustCommand(OuttakeSubsystem outtake) {
        this.outtake = outtake;
    }

    @Override
    public void initialize() {
        outtake.resetRollerEncoder();
    }

    @Override
    public void execute() {
        outtake.adjustRollers();
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(outtake.getRollerPosition()) >= outtake.getAdjustRotations().asDouble();
    }
}
