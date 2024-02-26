package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

import frc.robot.Units.Percent;

import java.util.function.DoubleSupplier;

public class OuttakeRollersManualCommand extends Command{
    private final OuttakeSubsystem outtake;
    private DoubleSupplier speed;

    public OuttakeRollersManualCommand(OuttakeSubsystem outtake, DoubleSupplier speed) {
        this.outtake = outtake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        outtake.moveRollers(new Percent(speed.getAsDouble() / 4));
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}