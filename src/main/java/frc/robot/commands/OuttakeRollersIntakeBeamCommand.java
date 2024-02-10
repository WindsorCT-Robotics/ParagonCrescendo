package frc.robot.commands;
import static frc.robot.Constants.OUTTAKE_MOTOR_TARGET_PERCENT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Units.Percent;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class OuttakeRollersIntakeBeamCommand extends Command {
    private final OuttakeSubsystem outtake;
    private final IntakeSubsystem intake;

    public OuttakeRollersIntakeBeamCommand(OuttakeSubsystem outtake, IntakeSubsystem intake) {
        this.outtake = outtake;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        outtake.moveRollers(new Percent(OUTTAKE_MOTOR_TARGET_PERCENT));
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.getBeamBreaker();
    }
}
