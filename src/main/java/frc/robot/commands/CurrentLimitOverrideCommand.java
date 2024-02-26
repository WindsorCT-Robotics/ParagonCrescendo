package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CurrentLimitOverrideCommand extends Command {
    private final DriveSubsystem drive;

    public CurrentLimitOverrideCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.setCurrentLimit(false);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setCurrentLimit(true);
    }
}
