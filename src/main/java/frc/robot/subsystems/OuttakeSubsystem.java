package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OuttakeSubsystem extends SubsystemBase {
    private CANSparkMax rollerMotor;
    private final DigitalInput beamBreaker;

    public OuttakeSubsystem() {
        rollerMotor = new CANSparkMax(OUTTAKE_ROLLER_MOTOR_CANID, MotorType.kBrushless);
        beamBreaker = new DigitalInput(OUTTAKE_BEAM_BREAKER_PIN);
    }

    @Override
    public void periodic() {
       SmartDashboard.putBoolean("Outtake Beam Breaker ", beamBreaker.get());
    }

    public void moveRollers(double speed) {
        rollerMotor.set(speed);
    }

    public boolean getBeamBreaker(){
        return beamBreaker.get();
    }
    
    public void stop() {
       rollerMotor.stopMotor();
    }
}                                                 