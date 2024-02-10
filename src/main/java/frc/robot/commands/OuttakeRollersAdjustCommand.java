package frc.robot.commands;
import static frc.robot.Constants.OUTTAKE_ADJUST_ROTATIONS;
import static frc.robot.Constants.OUTTAKE_ADJUST_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Units.Percent;
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
        outtake.moveRollers(new Percent(OUTTAKE_ADJUST_SPEED));
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(outtake.getRollerPosition()) >= OUTTAKE_ADJUST_ROTATIONS;
    }
}
