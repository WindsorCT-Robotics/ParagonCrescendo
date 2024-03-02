package frc.robot.commands.autos;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitDriveForwardAutoCommand extends SequentialCommandGroup {
    public WaitDriveForwardAutoCommand(DriveSubsystem drive) {
        addRequirements(drive);
        addCommands(
            new WaitCommand(8),
            new DriveCommand(() -> 0.35, () -> 0, drive).withTimeout(3)
        );
    }
}
