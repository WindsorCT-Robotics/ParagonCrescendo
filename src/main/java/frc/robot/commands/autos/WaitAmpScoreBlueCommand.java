package frc.robot.commands.autos;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitAmpScoreBlueCommand extends SequentialCommandGroup {
    private final DriveSubsystem drive;
    private final ArmSubsystem arm;
    private final OuttakeSubsystem outtake;

    public WaitAmpScoreBlueCommand(DriveSubsystem drive, ArmSubsystem arm, OuttakeSubsystem outtake) {
        this.drive = drive;
        this.arm = arm;
        this.outtake = outtake;
        addRequirements(this.drive, this.outtake, this.arm);
        addCommands(
            new WaitCommand(8),
            new AmpScoreAutoBlueCommand(drive, arm, outtake)
        );
    }
}
