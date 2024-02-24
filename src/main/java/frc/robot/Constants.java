package frc.robot;

public class Constants {

    public static final int OUTTAKE_MOTOR_MAX_RPM = 11000;
    public static final double OUTTAKE_MOTOR_GEAR_RATIO = (double) 1 / 16;
    public static final double OUTTAKE_MOTOR_TARGET_PERCENT = INTAKE_ROLLERS_TARGET_RPM / (OUTTAKE_MOTOR_MAX_RPM * OUTTAKE_MOTOR_GEAR_RATIO);

    public static final double OUTTAKE_ADJUST_DISTANCE = 2.5; // Inches
    public static final int OUTTAKE_ADJUST_DIRECTION = -1; // Too far is -1, not far enough is 1.
    public static final double OUTTAKE_ROLLERS_CIRCUMFERENCE = Math.PI * 1.67; // Outer diameter is 1.67"
    public static final double OUTTAKE_ADJUST_ROTATIONS = OUTTAKE_ADJUST_DISTANCE / (OUTTAKE_MOTOR_GEAR_RATIO * OUTTAKE_ROLLERS_CIRCUMFERENCE);
    public static final double OUTTAKE_ADJUST_SPEED = OUTTAKE_ADJUST_DIRECTION * 0.2;
}

