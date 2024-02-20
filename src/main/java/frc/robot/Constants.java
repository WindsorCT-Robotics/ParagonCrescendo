package frc.robot;

public class Constants {
    public static final String CAN_BUS_NAME = "rio";
    public static final int LEFT_MAIN_TALONFX = 1;
    public static final int LEFT_FOLLOWER_TALONFX = 2;
    public static final int RIGHT_MAIN_TALONFX = 3;
    public static final int RIGHT_FOLLOWER_TALONFX = 4;

    public static final int INTAKE_ROLLER_MOTOR_CANID = 5;
    public static final int INTAKE_BEAM_BREAKER_PIN = 0;    

    public static final class Arm {
        public static final int MOTOR_CANID = 6;
        public static final int ARM_HOME_LIMIT = 3;
        public static final int ARM_EXTEND_LIMIT = 4;

        public static final double ROTATION_SCALE = (25) * ((double) 44 / 18) * ((double) 44 / 18);
        public static final double ROTATION_VELOCITY = 0.8;
        public static final double ROTATION_CAP = ROTATION_SCALE * 1 / 3;
    }

    public static final int OUTTAKE_ROLLER_MOTOR_CANID = 7;
    public static final int OUTTAKE_BEAM_BREAKER_PIN = 1;

    public static final int INTAKE_ROLLERS_TARGET_RPM = 500;
    public static final int INTAKE_MOTOR_MAX_RPM = 5600;
    public static final double INTAKE_MOTOR_GEAR_RATIO = (double) 14 / 48;
    public static final double INTAKE_MOTOR_TARGET_PERCENT = INTAKE_ROLLERS_TARGET_RPM / (INTAKE_MOTOR_MAX_RPM * INTAKE_MOTOR_GEAR_RATIO);

    public static final int OUTTAKE_MOTOR_MAX_RPM = 11000;
    public static final double OUTTAKE_MOTOR_GEAR_RATIO = (double) 1 / 16;
    public static final double OUTTAKE_MOTOR_TARGET_PERCENT = INTAKE_ROLLERS_TARGET_RPM / (OUTTAKE_MOTOR_MAX_RPM * OUTTAKE_MOTOR_GEAR_RATIO);

    public static final double OUTTAKE_ADJUST_DISTANCE = 2.5; // Inches
    public static final int OUTTAKE_ADJUST_DIRECTION = -1; // Too far is -1, not far enough is 1.
    public static final double OUTTAKE_ROLLERS_CIRCUMFERENCE = Math.PI * 1.67; // Outer diameter is 1.67"
    public static final double OUTTAKE_ADJUST_ROTATIONS = OUTTAKE_ADJUST_DISTANCE / (OUTTAKE_MOTOR_GEAR_RATIO * OUTTAKE_ROLLERS_CIRCUMFERENCE);
    public static final double OUTTAKE_ADJUST_SPEED = OUTTAKE_ADJUST_DIRECTION * 0.2;
}

