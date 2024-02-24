package frc.robot.commands;

import frc.robot.subsystems.OuttakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeRollersBeamCommand extends Command {
    private final OuttakeSubsystem outtake;

    public OuttakeRollersBeamCommand(OuttakeSubsystem outtake) {
        this.outtake = outtake;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        outtake.outtakeRollers();
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return !outtake.isBeamBroken();
    }
}
