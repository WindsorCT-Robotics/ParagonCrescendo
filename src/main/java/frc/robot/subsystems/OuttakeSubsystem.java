package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.Percent;
import frc.robot.Units.Rotations;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OuttakeSubsystem extends SubsystemBase {
    private static final int ROLLER_MOTOR_CANID = 7;
    private static final int BEAM_BREAKER_PIN = 1;

    private CANSparkMax rollers;
    private final DigitalInput beamBreaker;
    private RelativeEncoder rollerEncoder;

    private static final int MOTOR_MAX_RPM = 11000;
    private static final double MOTOR_GEAR_RATIO = (double) 1 / 16; // Gear ratio is 4:1, 4:1

    private static double adjustDistance = 2.5; // Inches
    private static int adjustDirection = -1; // Too far is -1, not far enough is 1.
    private static final double ROLLERS_CIRCUMFERENCE = Math.PI * 1.67; // Outer diameter is 1.67"
    private static Rotations adjustRotations = new Rotations(adjustDistance / (MOTOR_GEAR_RATIO * ROLLERS_CIRCUMFERENCE));
    private static Percent adjustSpeed = new Percent(0.2);
    private static Percent adjustVelocity = new Percent(adjustDirection * adjustSpeed.asDouble());

    public OuttakeSubsystem() {
        rollers = new CANSparkMax(ROLLER_MOTOR_CANID, MotorType.kBrushless);
        rollers.setIdleMode(IdleMode.kBrake);
        rollers.setInverted(true);
        beamBreaker = new DigitalInput(BEAM_BREAKER_PIN);
        rollerEncoder = rollers.getEncoder();

        SmartDashboard.putNumber("Outtake Adjust Distance (inches)", adjustDistance);
        SmartDashboard.putNumber("Outtake Adjust Direction", adjustDirection);
        SmartDashboard.putNumber("Outtake Adjust Speed (percentage)", adjustSpeed.asDouble());
        SmartDashboard.putNumber("Outtake Percent Speed", IntakeSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake Beam Breaker ", beamBreaker.get());
        double tempAdjustDistance = SmartDashboard.getNumber("Outtake Adjust Distance (inches)", adjustDistance);
        if (adjustDistance != tempAdjustDistance) {
            adjustDistance = tempAdjustDistance;
            adjustRotations = new Rotations(adjustDistance / (MOTOR_GEAR_RATIO * ROLLERS_CIRCUMFERENCE));
        }
        
        int tempAdjustDirection = (int) SmartDashboard.getNumber("Outtake Adjust Direction", adjustDirection);
        double tempAdjustSpeed = SmartDashboard.getNumber("Outtake Adjust Speed (percentage)", adjustSpeed.asDouble());
        if (adjustDirection != tempAdjustDirection || adjustSpeed.asDouble() != tempAdjustSpeed) {
            adjustDirection = tempAdjustDirection;
            adjustSpeed = new Percent(tempAdjustSpeed);
            adjustVelocity = new Percent(adjustDirection * adjustSpeed.asDouble());
        }
    }

    public void moveRollers(Percent speed) {
        rollers.set(speed.asDouble());
    }

    public void outtakeRollers() {
        Percent speed = new Percent(IntakeSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
        rollers.set(speed.asDouble());
    }

    public void adjustRollers() {
        rollers.set(adjustVelocity.asDouble());
    }

    public boolean isBeamBroken() {
        return !beamBreaker.get();
    }

    public double getRollerPosition() {
        return rollerEncoder.getPosition();
    }

    public void resetRollerEncoder() {
        rollerEncoder.setPosition(0.0);
    }

    public Rotations getAdjustRotations() {
        return adjustRotations;
    }
    
    public void stop() {
       rollers.stopMotor();
    }
}