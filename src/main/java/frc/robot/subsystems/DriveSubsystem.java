package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.GearRatio;
import frc.robot.Units.Meters;
import frc.robot.Units.Radians;
import frc.robot.Units.Rotations;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class DriveSubsystem extends SubsystemBase {
    // CAN Bus Information
    private static final String CAN_BUS_NAME = "rio";
    private static final int LEFT_MAIN_TALONFX = 1;
    private static final int LEFT_FOLLOWER_TALONFX = 2;
    private static final int RIGHT_MAIN_TALONFX = 3;
    private static final int RIGHT_FOLLOWER_TALONFX = 4;
    private static final int PIGEON_ID = 10;

    // 4 Motor Controllers for Drivetrain
    private final TalonFX leftMain = new TalonFX(LEFT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX leftFollower = new TalonFX(LEFT_FOLLOWER_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightMain = new TalonFX(RIGHT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightFollower = new TalonFX(RIGHT_FOLLOWER_TALONFX, CAN_BUS_NAME);

    private final boolean isCoastMode = false;

    private final GearRatio gearRatio = new GearRatio(8.45d);
    private final Meters wheelDiameter = new Meters(0.1524);
    private final Radians wheelCircumference = new Radians(Math.PI * wheelDiameter.asDouble());

    // Feedforward/Feedback Gains
    public static final double ksVolts = 0.64469;
    public static final double kvVoltSecondsPerMeter = 1.6453;
    public static final double kaVoltSecondsSquaredPerMeter = 0.14978;
    public static final double kPDriveVel = 0.000007; //0.0000047

    // Differential Drive Kinematics
    public static final double kTrackwidthMeters = 0.5588;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max Trajector Velocity and Acceleration
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    // Ramsete Parameters
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Pigeon
    private Pigeon2 gyro;

    // Differential Drive object to build Drivetrain with
    private DifferentialDrive drive;

    // Odometry
    private final DifferentialDriveOdometry odometry;

    public DriveSubsystem() {
        initializeTalonFX(leftMain.getConfigurator(), "left");
        initializeTalonFX(leftFollower.getConfigurator(), "left");
        initializeTalonFX(rightMain.getConfigurator(), "right");
        initializeTalonFX(rightFollower.getConfigurator(), "right");

        leftFollower.setControl(new Follower(leftMain.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightMain.getDeviceID(), false));

        drive = new DifferentialDrive(leftMain, rightMain);
        gyro = new Pigeon2(PIGEON_ID);

        odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(),
            Meters.of(rotationsToMetersAsDouble(new Rotations(leftMain.getPosition().getValueAsDouble()))),
            Meters.of(rotationsToMetersAsDouble(new Rotations(rightMain.getPosition().getValueAsDouble()))));
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(),
            rotationsToMetersAsDouble(new Rotations(leftMain.getPosition().getValueAsDouble())),
            rotationsToMetersAsDouble(new Rotations(rightMain.getPosition().getValueAsDouble())));

        SmartDashboard.putNumber("Position/Left Main (m)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getPosition().getValueAsDouble())));
        SmartDashboard.putNumber("Position/Left Follower (m)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getPosition().getValueAsDouble())));
        SmartDashboard.putNumber("Position/Right Main (m)", -rotationsToMetersAsDouble(new Rotations(rightMain.getPosition().getValueAsDouble())));
        SmartDashboard.putNumber("Position/Right Follower (m)", -rotationsToMetersAsDouble(new Rotations(rightFollower.getPosition().getValueAsDouble())));

        SmartDashboard.putNumber("Velocity/Left Main (m-s)", -rotationsToMetersAsDouble(new Rotations(leftMain.getVelocity().getValueAsDouble())));
        SmartDashboard.putNumber("Velocity/Left Follower (m-s)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getVelocity().getValueAsDouble())));
        SmartDashboard.putNumber("Velocity/Right Main (m-s)", -rotationsToMetersAsDouble(new Rotations(rightMain.getVelocity().getValueAsDouble())));
        SmartDashboard.putNumber("Velocity/Right Follower (m-s)", -rotationsToMetersAsDouble(new Rotations(rightFollower.getVelocity().getValueAsDouble())));
        
        SmartDashboard.putNumber("Acceleration/Left Main (m-s-s)", -rotationsToMetersAsDouble(new Rotations(leftMain.getAcceleration().getValueAsDouble())));
        SmartDashboard.putNumber("Acceleration/Left Follower (m-s-s)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getAcceleration().getValueAsDouble())));
        SmartDashboard.putNumber("Acceleration/Right Main (m-s-s)", -rotationsToMetersAsDouble(new Rotations(rightMain.getAcceleration().getValueAsDouble())));
        SmartDashboard.putNumber("Acceleration/Right Follower (m-s-s)", -rotationsToMetersAsDouble(new Rotations(rightFollower.getAcceleration().getValueAsDouble())));

        SmartDashboard.putNumber("MotorTemperature/Left Main (C)", Math.round(leftMain.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Left Follower (C)", Math.round(leftFollower.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Right Main (C)", Math.round(rightMain.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Right Follower (C)", Math.round(rightFollower.getDeviceTemp().getValueAsDouble()));

        SmartDashboard.putNumber("MotorCurrent/Left Main", leftMain.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Left Follower", leftFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Right Main", rightMain.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Right Follower", rightFollower.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Coast Mode Enabled", isCoastMode);
    }

    private void initializeTalonFX(TalonFXConfigurator cfg, String side) {
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        if (side.equals("left")) {
            toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        
        if (isCoastMode) {
            toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        } else {
            toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
        toApply.CurrentLimits.StatorCurrentLimitEnable = true;
        toApply.CurrentLimits.StatorCurrentLimit = 90;
        cfg.apply(toApply);
        cfg.setPosition(0);
    }

    public void drive(double speed, double turn) {
        drive.curvatureDrive(speed, turn, true);
    }
    
    private double rotationsToMetersAsDouble(Rotations rotations) {
        return
            rotations.asMeters(gearRatio, wheelCircumference)
                     .asDouble();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(rotationsToMetersAsDouble(new Rotations(leftMain.getVelocity().getValueAsDouble())), 
                                                rotationsToMetersAsDouble(new Rotations(leftMain.getVelocity().getValueAsDouble())));
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            gyro.getRotation2d(),
            Meters.of(rotationsToMetersAsDouble(new Rotations(leftMain.getPosition().getValueAsDouble()))),
            Meters.of(rotationsToMetersAsDouble(new Rotations(rightMain.getPosition().getValueAsDouble()))),
            pose);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMain.setVoltage(leftVolts);
        rightMain.setVoltage(rightVolts);
        drive.feed();
    }

    public void resetEncoders() {
        leftMain.setPosition(0);
        rightMain.setPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (rotationsToMetersAsDouble(new Rotations(leftMain.getPosition().getValueAsDouble())) +
                rotationsToMetersAsDouble(new Rotations(rightMain.getPosition().getValueAsDouble()))) / 2.0;
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }  
    
    public double getTurnRate() {
        return -gyro.getRate();
    }

    public void stop() {
        leftMain.set(0);
        rightMain.set(0);
    }
}                                                 