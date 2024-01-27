package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveBeamBreakerCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public MoveBeamBreakerCommand(DriveSubsystem driveSubsystem,IntakeSubsystem intakeSubsystem) {
        this.driveSubsystem=driveSubsystem;
        this.intakeSubsystem=intakeSubsystem;
        addRequirements(driveSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveSubsystem.drive(.2,0);

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getBeamBreaker();
    }
}                             
