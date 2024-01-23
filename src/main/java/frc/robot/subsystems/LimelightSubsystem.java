package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{
    private NetworkTable limelight;
    private NetworkTableEntry tv;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    
    public LimelightSubsystem () {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LimelightV", (tv.getInteger(-1) == 1 ? true : false));
        SmartDashboard.putNumber("LimelightX", tx.getDouble(100.0));
        SmartDashboard.putNumber("LimelightY", ty.getDouble(100.0));
        SmartDashboard.putNumber("LimelightA", ta.getDouble(100.0));
    }

    public double getTX() {
        return tx.getDouble(0.0);
    }
}
