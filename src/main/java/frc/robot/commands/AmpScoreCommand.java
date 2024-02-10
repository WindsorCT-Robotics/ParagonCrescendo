package frc.robot.commands;

import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Units.Radians;

public class AmpScoreCommand extends SequentialCommandGroup {
    public AmpScoreCommand(ArmSubsystem arm, OuttakeSubsystem outtake) {
        addCommands(
            Commands.runOnce(() -> {arm.setGoal(new Radians(Arm.ROTATION_CAP)); arm.enable();}, arm),
            new OuttakeRollersBeamCommand(outtake),
            Commands.runOnce(() -> {arm.setGoal(new Radians(Arm.ROTATION_OFFSET)); arm.enable();}, arm)
        );
    }
}
