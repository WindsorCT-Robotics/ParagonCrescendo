package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class StopRollersCommand extends Command {
    private final OuttakeSubsystem outtake;
    private final IntakeSubsystem intake;

    public StopRollersCommand(OuttakeSubsystem outtake, IntakeSubsystem intake) {
        this.outtake = outtake;
        this.intake = intake;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

