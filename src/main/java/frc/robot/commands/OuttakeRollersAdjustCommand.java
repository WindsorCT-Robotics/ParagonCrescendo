package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

public class OuttakeRollersAdjustCommand extends Command {
    private final OuttakeSubsystem outtake;
    private double startingEncoderPosition;

    public OuttakeRollersAdjustCommand(OuttakeSubsystem outtake) {
        this.outtake = outtake;
    }

    @Override
    public void initialize() {
        startingEncoderPosition = outtake.getRollerPosition();
        System.out.println(startingEncoderPosition);
    }

    @Override
    public void execute() {
        outtake.adjustRollers();
        System.out.println(outtake.getRollerPosition());
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Math.abs(outtake.getRollerPosition()) - Math.abs(startingEncoderPosition)) >= outtake.getAdjustRotations().asDouble();
    }
}
