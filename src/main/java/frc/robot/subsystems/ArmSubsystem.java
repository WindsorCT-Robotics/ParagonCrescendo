package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Units.Radians;
import frc.robot.Units.Rotations;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax armMotor;
    private final ArmFeedforward armController;
    private final RelativeEncoder armEncoder;

    public ArmSubsystem() {
        super(new ProfiledPIDController(Arm.PID_kP, 0, 0, new TrapezoidProfile.Constraints(Arm.ROTATION_VELOCITY_CAP, Arm.ROTATION_ACCELERATION_CAP)), 0);
        armMotor = new CANSparkMax(Arm.MOTOR_CANID, MotorType.kBrushless);
        armController = new ArmFeedforward(Arm.ROTATION_kS, Arm.ROTATION_kG, Arm.ROTATION_kV, Arm.ROTATION_kA);
        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);
        setGoal(new Radians(Arm.ROTATION_OFFSET));
    }

    @Override
    public void periodic() {

    }

    public void setGoal(Radians radians) { setGoal(radians.asDouble()); }

    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = armController.calculate(setpoint.position, setpoint.velocity);

        armMotor.setVoltage(output + feedforward);
    }

    public Radians getRotationsInRadians() { 
        return new Radians(new Rotations(armEncoder.getPosition()), Arm.ROTATION_OFFSET);
    }

    public double getMeasurement() {
        return getRotationsInRadians().asDouble();
    }

    public void stop() {
       armMotor.stopMotor();
    }
}                                                 
