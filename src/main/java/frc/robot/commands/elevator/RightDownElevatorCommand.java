package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Units.Percent;
import frc.robot.subsystems.ElevatorSubsystem;

public class RightDownElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier speed;

    public RightDownElevatorCommand(ElevatorSubsystem elevator, DoubleSupplier speed) {
        this.elevator = elevator;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.moveRightMotor(new Percent(-speed.getAsDouble()*0.95));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.rightStop();
    }

    @Override
    public boolean isFinished() {
        return elevator.isRightAtBottom();
    }
}