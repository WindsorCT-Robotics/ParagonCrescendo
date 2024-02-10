package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class IntakeNoteCommand {
    private final IntakeSubsystem intake;
    private final OuttakeSubsystem outtake;

    public IntakeNoteCommand(IntakeSubsystem intake, OuttakeSubsystem outtake) {
        this.intake = intake;
        this.outtake = outtake;
    }

    
}
