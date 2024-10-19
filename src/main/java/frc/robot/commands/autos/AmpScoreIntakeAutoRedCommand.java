package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AmpScoreCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class AmpScoreIntakeAutoRedCommand extends SequentialCommandGroup {
    DriveSubsystem drive;
    ArmSubsystem arm;
    OuttakeSubsystem outtake;
    IntakeSubsystem intake;

    public AmpScoreIntakeAutoRedCommand(DriveSubsystem drive, ArmSubsystem arm, OuttakeSubsystem outtake, IntakeSubsystem intake) {
        addRequirements(drive);
        this.drive = drive;
        this.arm = arm;
        this.outtake = outtake;
        this.intake = intake;

        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveSubsystem.ksVolts,
                DriveSubsystem.kvVoltSecondsPerMeter,
                DriveSubsystem.kaVoltSecondsSquaredPerMeter),
            DriveSubsystem.kDriveKinematics,
            10);

        // Create config for trajectory
        TrajectoryConfig initialConfig =
            new TrajectoryConfig(
                    DriveSubsystem.kMaxSpeedMetersPerSecond,
                    DriveSubsystem.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveSubsystem.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);

        Trajectory initialDriveTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1.28, -0.5, new Rotation2d(70*Math.PI/180)), //-0.508, -0.451
                // Pass config
                initialConfig);

        RamseteCommand initialDriveCommand =
            new RamseteCommand(
                initialDriveTrajectory,
                drive::getPose,
                new RamseteController(DriveSubsystem.kRamseteB, DriveSubsystem.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveSubsystem.ksVolts,
                    DriveSubsystem.kvVoltSecondsPerMeter,
                    DriveSubsystem.kaVoltSecondsSquaredPerMeter),
                DriveSubsystem.kDriveKinematics,
                drive::getWheelSpeeds,
                new PIDController(DriveSubsystem.kPDriveVel, 0, 0),
                new PIDController(DriveSubsystem.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drive::tankDriveVolts,
                drive);


        // Create config for trajectory
        TrajectoryConfig intakeConfig =
            new TrajectoryConfig(
                    DriveSubsystem.kMaxSpeedMetersPerSecond,
                    DriveSubsystem.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveSubsystem.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        Trajectory intakeDriveTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(70*Math.PI/180)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1,0),new Translation2d(2, 0.2),new Translation2d(4, 0.25)),//-0.254, 0.315
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(6 ,0.9, new Rotation2d(0*Math.PI/180)), //-7.366, 0.315
                // Pass config
                intakeConfig);

        RamseteCommand intakeDriveCommand =
            new RamseteCommand(
                intakeDriveTrajectory,
                drive::getPose,
                new RamseteController(DriveSubsystem.kRamseteB, DriveSubsystem.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveSubsystem.ksVolts,
                    DriveSubsystem.kvVoltSecondsPerMeter,
                    DriveSubsystem.kaVoltSecondsSquaredPerMeter),
                DriveSubsystem.kDriveKinematics,
                drive::getWheelSpeeds,
                new PIDController(DriveSubsystem.kPDriveVel, 0, 0),
                new PIDController(DriveSubsystem.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drive::tankDriveVolts,
                drive);
        
        addCommands(Commands.runOnce(() -> drive.resetOdometry(initialDriveTrajectory.getInitialPose())),
                    initialDriveCommand,
                    Commands.runOnce(() -> drive.tankDriveVolts(0, 0)),
                    new AmpScoreCommand(arm, outtake),
                    Commands.runOnce(() -> drive.resetOdometry(intakeDriveTrajectory.getInitialPose())),
                    new ParallelDeadlineGroup(new IntakeNoteCommand(intake, outtake),
                                              intakeDriveCommand));
    }
}
