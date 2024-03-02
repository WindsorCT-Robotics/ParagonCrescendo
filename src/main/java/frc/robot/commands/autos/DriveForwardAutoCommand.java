package frc.robot.commands.autos;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveForwardAutoCommand extends SequentialCommandGroup {
    public DriveForwardAutoCommand(DriveSubsystem drive) {
        addRequirements(drive);
        addCommands(
            new DriveCommand(() -> 0.35, () -> 0, drive).withTimeout(3)        );
    }
}
