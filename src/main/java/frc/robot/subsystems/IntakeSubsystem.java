package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    
    private final DigitalInput beamBreaker;

    public IntakeSubsystem() {
        beamBreaker = new DigitalInput(INTAKE_BEAMBREAKER);
    }

    @Override
    public void periodic() {
       SmartDashboard.putBoolean("Beam Breaker ", beamBreaker.get());
    }
    
    public void stop() {
       
    }

    public boolean getBeamBreaker(){
        return beamBreaker.get();
    }
}                                                 