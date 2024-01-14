package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private DoubleSupplier speed;
    private double speedScale = .80;
    private double positiveSpeedRateLimit = 5.0;
    private double negativeSpeedRateLimit = -5.0;
    private SlewRateLimiter speedLimiter = new SlewRateLimiter(positiveSpeedRateLimit, negativeSpeedRateLimit, 0);
    
    private DoubleSupplier turn;
    private double turnScale = .250;
    private double positiveTurnRateLimit = 10.0;
    private double negativeTurnRateLimit = -10.0;
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(positiveTurnRateLimit, negativeTurnRateLimit, 0);

    private DriveSubsystem driveSubsystem;

    public DriveCommand(DoubleSupplier speed, DoubleSupplier turn, DriveSubsystem driveSubsystem) {
        this.speed = speed;
        this.turn = turn;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stop();
    }

    @Override
    public void execute() {
        double driveSpeed = speed.getAsDouble() * speedScale * Math.abs(speed.getAsDouble());
        double turnSpeed = turn.getAsDouble() * turnScale * Math.abs(turn.getAsDouble());

        driveSubsystem.drive(speedLimiter.calculate(driveSpeed), turnLimiter.calculate(turnSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}                             
