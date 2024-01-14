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

public class DriveSubsystem extends SubsystemBase {
    // 4 Motor Controllers for Drivetrain
    private final TalonFX leftMain = new TalonFX(LEFT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX leftFollower = new TalonFX(LEFT_FOLLOWER_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightMain = new TalonFX(RIGHT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightFollower = new TalonFX(RIGHT_FOLLOWER_TALONFX, CAN_BUS_NAME);

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

    private void initializeTalonFX(TalonFXConfigurator cfg, String side) {
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        if (side.equals("left")) {
            toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
       
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.apply(toApply);
        cfg.setPosition(0);
    }

    public void drive(double speed, double turn) {
        drive.curvatureDrive(speed, turn, true);
    }           
    
    public void stop() {
        leftMain.set(0);
        rightMain.set(0);
    }
}                                                 