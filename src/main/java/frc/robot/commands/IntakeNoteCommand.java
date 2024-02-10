package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeNoteCommand extends SequentialCommandGroup{
    public IntakeNoteCommand(IntakeSubsystem intake, OuttakeSubsystem outtake) {
        addCommands(
            new IntakeRollersNoBeamCommand(intake),
            new ParallelDeadlineGroup(
                new IntakeRollersBeamCommand(intake),
                new OuttakeRollersBeamCommand(outtake, intake)
            ),
            new OuttakeRollersAdjustCommand(outtake)
        );
    }
}
