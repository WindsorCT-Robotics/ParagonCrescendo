package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class CenterAprilTagCommand extends Command{
    private DriveSubsystem driveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private double limelightTX = 0.0;
    private double currentHeading = 0.0;
    private double angleTurn = 0.0;
    
    public CenterAprilTagCommand(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(driveSubsystem);
        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stop();
        limelightTX = limelightSubsystem.getTX();
        currentHeading = driveSubsystem.getAngle();
        SmartDashboard.putNumber("angleTurn", angleTurn);
        SmartDashboard.putNumber("currentHeading", currentHeading);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(0.0, -Math.signum(limelightTX) * 0.05);
        SmartDashboard.putNumber("driveSubsystem.getAngle()", driveSubsystem.getAngle());
        SmartDashboard.putString("comparison", (angleTurn < 0 ? ">=" : "<="));
    }

    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public boolean isFinished() {
        if (limelightTX < 0) {
            return limelightSubsystem.getTX() >= 0;
        }
        return limelightSubsystem.getTX() <= 0;
    }
}
