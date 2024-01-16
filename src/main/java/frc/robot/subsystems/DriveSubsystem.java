package frc.robot.subsystems;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
    // 4 Motor Controllers for Drivetrain
    private final TalonFX leftMain = new TalonFX(LEFT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX leftFollower = new TalonFX(LEFT_FOLLOWER_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightMain = new TalonFX(RIGHT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightFollower = new TalonFX(RIGHT_FOLLOWER_TALONFX, CAN_BUS_NAME);

    private final boolean brakeMode = true; // true is Brake, false is Coast

    // Differential Drive object to build Drivetrain with
    private DifferentialDrive drive;

    public DriveSubsystem() {
        initializeTalonFX(leftMain.getConfigurator(), "left");
        initializeTalonFX(leftFollower.getConfigurator(), "left");
        initializeTalonFX(rightMain.getConfigurator(), "right");
        initializeTalonFX(rightFollower.getConfigurator(), "right");

        leftFollower.setControl(new Follower(leftMain.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightMain.getDeviceID(), false));

        drive = new DifferentialDrive(leftMain, rightMain);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position/Left Main (m)", -rotationsToMeters(leftMain.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Position/Left Follower (m)", -rotationsToMeters(leftFollower.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Position/Right Main (m)", -rotationsToMeters(rightMain.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Position/Right Follower (m)", -rotationsToMeters(rightFollower.getPosition().getValueAsDouble()));

        SmartDashboard.putNumber("Velocity/Left Main (m-s)", -rotationsToMeters(leftMain.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("Velocity/Left Follower (m-s)", -rotationsToMeters(leftFollower.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("Velocity/Right Main (m-s)", -rotationsToMeters(rightMain.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("Velocity/Right Follower (m-s)", -rotationsToMeters(rightFollower.getVelocity().getValueAsDouble()));
        
        SmartDashboard.putNumber("Acceleration/Left Main (m-s-s)", -rotationsToMeters(leftMain.getAcceleration().getValueAsDouble()));
        SmartDashboard.putNumber("Acceleration/Left Follower (m-s-s)", -rotationsToMeters(leftFollower.getAcceleration().getValueAsDouble()));
        SmartDashboard.putNumber("Acceleration/Right Main (m-s-s)", -rotationsToMeters(rightMain.getAcceleration().getValueAsDouble()));
        SmartDashboard.putNumber("Acceleration/Right Follower (m-s-s)", -rotationsToMeters(rightFollower.getAcceleration().getValueAsDouble()));

        SmartDashboard.putNumber("MotorTemperature/Left Main (C)", Math.round(leftMain.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Left Follower (C)", Math.round(leftFollower.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Right Main (C)", Math.round(rightMain.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Right Follower (C)", Math.round(rightFollower.getDeviceTemp().getValueAsDouble()));

        SmartDashboard.putNumber("MotorCurrent/Left Main", leftMain.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Left Follower", leftFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Right Main", rightMain.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Right Follower", rightFollower.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Brake Mode", brakeMode);
    }

    private void initializeTalonFX(TalonFXConfigurator cfg, String side) {
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        if (side.equals("left")) {
            toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        
        if (brakeMode) {
            toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        
        cfg.apply(toApply);
        cfg.setPosition(0);
    }

    public void drive(double speed, double turn) {
        drive.curvatureDrive(speed, turn, true);
    }   
    
    private static double rotationsToMeters(double rotations) {
        final double gearRatio = 8.45; // 8.45:1 gear ratio
        final double wheelDiameter = 0.1524; // 6-inch wheel diameter in meters
        final double wheelCircumference = (Math.PI * wheelDiameter);

        return rotations / gearRatio * wheelCircumference;
    }
    
    public void stop() {
        leftMain.set(0);
        rightMain.set(0);
    }
}                                                 