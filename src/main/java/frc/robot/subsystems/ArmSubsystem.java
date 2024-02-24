package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Units.Percent;
import frc.robot.Units.Rotations;
import frc.robot.Exceptions.ArmMovementException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private static final int MOTOR_CANID = 6;
    private static final int HOME_LIMIT_PIN = 3;
    // private static final int EXTEND_LIMIT_PIN = 4;

    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final DigitalInput armHomeLimit;

    private static final double ROTATION_SCALE = 25 * Math.pow((44d/18d), 2); // Gear ratio is 5:1, 5:1, 44:18, 44:18
    private static final Rotations ROTATION_CAP = new Rotations(ROTATION_SCALE / 2.6d); // Arm should never go more than 1/2.6 of a turn
    private static final Percent ROTATION_POWER = new Percent(0.8d);    

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
        armMotor = new CANSparkMax(MOTOR_CANID, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setInverted(true);
        armEncoder = armMotor.getEncoder();
        armState = ArmState.UNKNOWN;
        armHomeLimit = new DigitalInput(HOME_LIMIT_PIN);
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
        armMotor.set(-ROTATION_POWER.asDouble());
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
                armMotor.set(ROTATION_POWER.asDouble());
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
                armMotor.set(-ROTATION_POWER.asDouble());
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
