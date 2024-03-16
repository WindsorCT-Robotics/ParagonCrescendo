package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Units.Percent;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int LEFT_MOTOR_CANID = 8;
    private static final int RIGHT_MOTOR_CANID = 9;
    private static final int RIGHT_UPPER_LIMIT_PIN = 4;
    private static final int LEFT_UPPER_LIMIT_PIN = 5;
    private static final int RIGHT_LOWER_LIMIT_PIN = 6;
    private static final int LEFT_LOWER_LIMIT_PIN = 7;

    private final CANSparkMax leftElev;
    private final CANSparkMax rightElev;

    private final DigitalInput rightUpperLimit;
    private final DigitalInput leftUpperLimit;
    private final DigitalInput rightLowerLimit;
    private final DigitalInput leftLowerLimit;

    public ElevatorSubsystem() {
        leftElev = new CANSparkMax(LEFT_MOTOR_CANID, MotorType.kBrushless);
        rightElev = new CANSparkMax(RIGHT_MOTOR_CANID, MotorType.kBrushless);

        leftElev.setInverted(true);
        leftElev.setIdleMode(IdleMode.kBrake);
        rightElev.setIdleMode(IdleMode.kBrake);

        rightUpperLimit = new DigitalInput(RIGHT_UPPER_LIMIT_PIN);
        leftUpperLimit = new DigitalInput(LEFT_UPPER_LIMIT_PIN);
        rightLowerLimit = new DigitalInput(RIGHT_LOWER_LIMIT_PIN);
        leftLowerLimit = new DigitalInput(LEFT_LOWER_LIMIT_PIN);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Right Upper Limit Switch", rightUpperLimit.get());
        SmartDashboard.putBoolean("Left Upper Limit Switch", leftUpperLimit.get());
        SmartDashboard.putBoolean("Right Lower Limit Switch", rightLowerLimit.get());
        SmartDashboard.putBoolean("Left Lower Limit Switch", leftLowerLimit.get());
    }

    public void moveLeftMotor(Percent speed) {
        leftElev.set(speed.asDouble());
    }

    public void moveRightMotor(Percent speed) {
        rightElev.set(speed.asDouble());
    }

    public boolean isRightAtTop() {
        return rightUpperLimit.get();
    }

    public boolean isLeftAtTop() {
        return leftUpperLimit.get();
    }

    public boolean isRightAtBottom() {
        return rightLowerLimit.get();
    }

    public boolean isLeftAtBottom() {
        return leftLowerLimit.get();
    }

    public void leftStop() {
        leftElev.stopMotor();
    }

    public void rightStop() {
        rightElev.stopMotor();
    }
}
