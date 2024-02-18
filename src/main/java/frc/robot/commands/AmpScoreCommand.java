package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpScoreCommand extends SequentialCommandGroup {
    public AmpScoreCommand(ArmSubsystem arm, OuttakeSubsystem outtake) {
        addCommands(
            new ExtendArmCommand(arm),
            new OuttakeRollersBeamCommand(outtake),
            new RetractArmCommand(arm)
        );
    }
}
