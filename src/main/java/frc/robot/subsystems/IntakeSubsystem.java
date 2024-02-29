package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.Percent;
import frc.robot.Units.RotationsPerMinute;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    private static final int ROLLER_MOTOR_CANID = 5;
    private static final int BEAM_BREAKER_PIN = 0;

    private final CANSparkMax rollers;
    private final DigitalInput beamBreaker;

    private static final int MOTOR_MAX_RPM = 5600;
    private static final double MOTOR_GEAR_RATIO = (double) 14 / 48;
    public static RotationsPerMinute targetRPM = new RotationsPerMinute(300, 1);

    public IntakeSubsystem() {
        rollers = new CANSparkMax(ROLLER_MOTOR_CANID, MotorType.kBrushless);
        beamBreaker = new DigitalInput(BEAM_BREAKER_PIN);

        SmartDashboard.putNumber("Intake Target RPM", targetRPM.asDouble());
        SmartDashboard.putNumber("Intake Percent Speed",targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
    }

    @Override
    public void periodic() {
       SmartDashboard.putBoolean("Intake Beam Breaker ", beamBreaker.get());

       targetRPM = new RotationsPerMinute(SmartDashboard.getNumber("Intake Target RPM", targetRPM.asDouble()), 1);
    }

    public void moveRollers(Percent speed) {
        rollers.set(speed.asDouble());
    }

    public void intakeRollers() {
        Percent speed = new Percent(targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
        rollers.set(speed.asDouble());
    }

    public boolean isBeamBroken(){
        return !beamBreaker.get();
    }
    
    public void stop() {
       rollers.stopMotor();
    }    
}                                                 