package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollersNoBeamCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeRollersNoBeamCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intakeRollers();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.isBeamBroken();
    }
}
