package frc.robot.commands.autos;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveForwardAutoCommand extends SequentialCommandGroup {
    private final DriveSubsystem drive;

    public DriveForwardAutoCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(this.drive);
        addCommands(
            new DriveCommand(() -> 0.35, () -> 0, this.drive).withTimeout(4)
        );
    }
}
