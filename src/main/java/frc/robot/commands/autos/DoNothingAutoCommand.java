package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DoNothingAutoCommand extends SequentialCommandGroup {
    public DoNothingAutoCommand() {
        addCommands(
            new WaitCommand(12)
        );
    }
}
