package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.Units.Percent;

import java.util.function.DoubleSupplier;

public class IntakeRollersManualCommand extends Command{
    private final IntakeSubsystem intake;
    private DoubleSupplier speed;

    public IntakeRollersManualCommand(IntakeSubsystem intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.moveRollers(new Percent(speed.getAsDouble() / 4));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}