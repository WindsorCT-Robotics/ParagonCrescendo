package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.Percent;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OuttakeSubsystem extends SubsystemBase {
    private CANSparkMax rollerMotor;
    private final DigitalInput beamBreaker;
    private RelativeEncoder rollerEncoder;

    public OuttakeSubsystem() {
        rollerMotor = new CANSparkMax(OUTTAKE_ROLLER_MOTOR_CANID, MotorType.kBrushless);
        beamBreaker = new DigitalInput(OUTTAKE_BEAM_BREAKER_PIN);
        rollerEncoder = rollerMotor.getEncoder();
    }

    @Override
    public void periodic() {
       SmartDashboard.putBoolean("Outtake Beam Breaker ", beamBreaker.get());
    }

    public void moveRollers(Percent speed) {
        rollerMotor.set(speed.asDouble());
    }

    /* BeamBreaker is true when beam is unbroken (open)
     * BeamBreaker is false when beam is broken (closed)
     */
    public boolean getBeamBreaker(){
        return beamBreaker.get();
    }

    public double getRollerPosition() {
        return rollerEncoder.getPosition();
    }

    public void resetRollerEncoder() {
        rollerEncoder.setPosition(0.0);
    }
    
    public void stop() {
       rollerMotor.stopMotor();
    }
}                                                 