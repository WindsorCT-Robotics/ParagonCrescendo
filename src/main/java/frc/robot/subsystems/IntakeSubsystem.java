package frc.robot.subsystems;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.Percent;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax rollers;
    private final DigitalInput beamBreaker;

    public IntakeSubsystem() {
        rollers = new CANSparkMax(INTAKE_ROLLER_MOTOR_CANID, MotorType.kBrushless);
        beamBreaker = new DigitalInput(INTAKE_BEAMBREAKER);
    }

    @Override
    public void periodic() {
       SmartDashboard.putBoolean("Intake Beam Breaker ", beamBreaker.get());
    }

    public void moveRollers(Percent speed) {
        rollers.set(speed.asDouble());
    }

    /* BeamBreaker is true when beam is unbroken (open)
     * BeamBreaker is false when beam is broken (closed)
     */
    public boolean getBeamBreaker(){
        return beamBreaker.get();
    }
    
    public void stop() {
       rollers.stopMotor();
    }    
}                                                 