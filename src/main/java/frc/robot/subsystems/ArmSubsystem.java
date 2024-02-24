package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Units.Percent;
import frc.robot.Units.Rotations;
import frc.robot.Exceptions.ArmMovementException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final DigitalInput armHomeLimit;

    private static final double ROTATION_SCALE = 25 * Math.pow((44d/18d), 2); // TODO: Ask Mr.G to break down this formula and eliminate all magic numbers
    private static final Rotations ROTATION_CAP = new Rotations(ROTATION_SCALE / 2.6d);
    private static final Percent ARM_ROTATION_POWER = new Percent(0.8d);    

    public enum ArmState {
        UNKNOWN,
        HOMING,
        RETRACTED,
        EXTENDED,
        RETRACTING,
        EXTENDING
    }

    private ArmState armState;

    public ArmSubsystem() {
        armMotor = new CANSparkMax(Arm.MOTOR_CANID, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armState = ArmState.UNKNOWN;
        armHomeLimit = new DigitalInput(Arm.ARM_HOME_LIMIT_PIN);
    }
    
    private boolean isAtLimit() {
        return !armHomeLimit.get();
    }

    @Override
    public void periodic() {
        switch (armState) {
            case UNKNOWN:
            case EXTENDED:
            case RETRACTED:
                break;

            case EXTENDING:
                if (armEncoder.getPosition() >= ROTATION_CAP.asDouble()) {
                    armMotor.stopMotor();
                    armState = ArmState.EXTENDED;
                }
                break;

            case HOMING:
            case RETRACTING:
                if (isAtLimit()) {
                    armMotor.stopMotor();
                    if (armState == ArmState.HOMING) {
                        armEncoder.setPosition(0);
                    }
                    armState = ArmState.RETRACTED;
                }
                break;
        }

        SmartDashboard.putBoolean("Home Position Switch", isAtLimit());
        SmartDashboard.putNumber("Max Travel Distance", ROTATION_CAP.asDouble());
        SmartDashboard.putNumber("Current Travel Distance", armEncoder.getPosition());
        SmartDashboard.putString("Arm State", armState.toString());
    }

    public ArmState getArmState() {
        return armState;
    }

    public void home () {
        armMotor.set(-ARM_ROTATION_POWER.asDouble());
        armState = ArmState.HOMING;
    }
    
    public void extend() throws ArmMovementException {
        switch(armState) {
            case UNKNOWN:
            case HOMING:
                throw new ArmMovementException(armState, ArmState.EXTENDING);

            case EXTENDING:
            case EXTENDED:
            break;

            case RETRACTING:
            case RETRACTED:
                armMotor.set(ARM_ROTATION_POWER.asDouble());
                armState = ArmState.EXTENDING;
            break;
        }
    }

    public void retract() throws ArmMovementException {
        switch(armState) {
            case UNKNOWN:
            case HOMING:
                throw new ArmMovementException(armState, ArmState.RETRACTING);

            case EXTENDING:
            case EXTENDED:
                armMotor.set(-ARM_ROTATION_POWER.asDouble());
                armState = ArmState.RETRACTING;
            break;

            case RETRACTING:
            case RETRACTED:
            break;
        }
    }

    public void cancel() {
       armMotor.stopMotor();
       armState = ArmState.UNKNOWN;
    }
}                                                 
