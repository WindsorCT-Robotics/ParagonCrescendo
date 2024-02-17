package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Units.Radians;
import frc.robot.Units.Rotations;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final DigitalInput armHomeLimit = new DigitalInput(Arm.ARM_HOME_LIMIT);
    private final DigitalInput armExtendLimit = new DigitalInput(Arm.ARM_EXTEND_LIMIT);


    public enum ArmState {
        INSIDE,
        OUTSIDE
    }

    private ArmState armState;

    public ArmSubsystem() {
        armMotor = new CANSparkMax(Arm.MOTOR_CANID, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);
        armState = ArmState.INSIDE;
    }

    @Override
    public void periodic() {

    }

    public void moveArm(double speed) {
        armMotor.set(speed);
    }

    public double getEncoderPosition() {
        return armEncoder.getPosition();
    }

    public boolean isHome() {
        return armHomeLimit.get();
    }

    public boolean isFullyExtended() {
        return armExtendLimit.get();
    }

    public ArmState getArmState() {
        return armState;
    }

    public void setInsideArmState() {
        armState = ArmState.INSIDE;
    }

    public void setOutsideArmState() {
        armState = ArmState.OUTSIDE;
    }

    public void stop() {
       armMotor.stopMotor();
    }
}                                                 
