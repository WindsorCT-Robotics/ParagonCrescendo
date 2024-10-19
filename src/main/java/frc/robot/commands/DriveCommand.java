package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends Command {
    private final DoubleSupplier speed;
    private double speedScale = .825; // .825
    private double positiveSpeedRateLimit = 5.0;
    private double negativeSpeedRateLimit = -5.0;
    private SlewRateLimiter speedLimiter = new SlewRateLimiter(positiveSpeedRateLimit, negativeSpeedRateLimit, 0);
    
    private final DoubleSupplier turn;
    private double turnScale = .275; // .275
    private double positiveTurnRateLimit = 10.0;
    private double negativeTurnRateLimit = -10.0;
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(positiveTurnRateLimit, negativeTurnRateLimit, 0);

    private boolean squaredInputs = true;

    private final DriveSubsystem driveSubsystem;

    public DriveCommand(DoubleSupplier speed, DoubleSupplier turn, DriveSubsystem driveSubsystem) {
        this.speed = speed;
        this.turn = turn;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stop();

        SmartDashboard.putNumber("Speed Scale", speedScale);
        SmartDashboard.putNumber("Positive Speed Rate Limit", positiveSpeedRateLimit);
        SmartDashboard.putNumber("Negative Speed Rate Limit", negativeSpeedRateLimit);
        
        SmartDashboard.putNumber("Turn Scale", turnScale);
        SmartDashboard.putNumber("Positive Turn Rate Limit", positiveTurnRateLimit);
        SmartDashboard.putNumber("Negative Turn Rate Limit", negativeTurnRateLimit);

        SmartDashboard.putBoolean("Squaring Inputs", squaredInputs);

    }

    @Override
    public void execute() {
        speedScale = SmartDashboard.getNumber("Speed Scale", speedScale);
        double newPositiveSpeedRateLimit = SmartDashboard.getNumber("Positive Speed Rate Limit", positiveSpeedRateLimit);
        double newNegativeSpeedRateLimit = SmartDashboard.getNumber("Negative Speed Rate Limit", negativeSpeedRateLimit);
        
        turnScale = SmartDashboard.getNumber("Turn Scale", turnScale);
        double newPositiveTurnRateLimit = SmartDashboard.getNumber("Positive Turn Rate Limit", positiveTurnRateLimit);
        double newNegativeTurnRateLimit = SmartDashboard.getNumber("Negative Turn Rate Limit", negativeTurnRateLimit);

        squaredInputs = SmartDashboard.getBoolean("Squaring Inputs", squaredInputs);

        if (newPositiveSpeedRateLimit != positiveSpeedRateLimit || newNegativeSpeedRateLimit != negativeSpeedRateLimit) {
            speedLimiter = new SlewRateLimiter(positiveSpeedRateLimit, negativeSpeedRateLimit, 0);
        }
        if (newPositiveTurnRateLimit != positiveTurnRateLimit || newNegativeTurnRateLimit != negativeTurnRateLimit) {
            turnLimiter = new SlewRateLimiter(positiveTurnRateLimit, negativeTurnRateLimit, 0);
        }

        double driveSpeed = speed.getAsDouble() * speedScale * (squaredInputs ? Math.abs(speed.getAsDouble()) : 1);
        double turnSpeed = turn.getAsDouble() * turnScale * (squaredInputs ? Math.abs(turn.getAsDouble()) : 1);

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
