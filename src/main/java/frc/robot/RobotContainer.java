// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private DriveSubsystem drive;
  private CommandXboxController driveController;
  private CommandXboxController operatorController;

  private IntakeSubsystem intake;
  private ArmSubsystem arm;
  private OuttakeSubsystem outtake;
  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    drive = new DriveSubsystem();
    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    drive.setDefaultCommand(new DriveCommand(() -> -driveController.getLeftY(), () -> -driveController.getRightX(), drive));

    intake = new IntakeSubsystem();
    arm = new ArmSubsystem();
    outtake = new OuttakeSubsystem();

    m_chooser.setDefaultOption("Drive Forward Auto Command", new DriveForwardAutoCommand(drive));
    m_chooser.addOption("Wait Drive Forward Auto Command", new WaitDriveForwardAutoCommand(drive));
    m_chooser.addOption("Do Nothing Auto Command", new DoNothingAutoCommand());
    m_chooser.addOption("Amp Score Auto Command", new AmpScoreAutoCommand(drive, arm, outtake));
    m_chooser.addOption("Amp Score and Intake Auto Command", new AmpScoreIntakeAutoCommand(drive, arm, outtake, intake));
    SmartDashboard.putData("Auto Mode", m_chooser);

    configureButtonBindings();
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public void armHomeIfUnknown(CommandScheduler cmd){
    if (arm.getArmState() == ArmSubsystem.ArmState.UNKNOWN) {
      cmd.schedule(new HomeArmCommand(arm));
    }
  }



  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Intake note from ground
    driveController.leftBumper().onTrue(new IntakeNoteCommand(intake, outtake).until(operatorController.b()));

    // Amp Score with arm moving
    driveController.rightBumper().onTrue(new AmpScoreCommand(arm, outtake));

    // Move arm up and down
    operatorController.povUp().onTrue(new ExtendArmCommand(arm));
    operatorController.povDown().onTrue(new RetractArmCommand(arm));

    // Manually control intake rollers
    Trigger opLeftJoy = new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.2);
    opLeftJoy.whileTrue(new IntakeRollersManualCommand(intake, () -> -operatorController.getLeftY()));

    // Manually control outtake rollers
    Trigger opRightJoy = new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.2);
    opRightJoy.whileTrue(new OuttakeRollersManualCommand(outtake, () -> -operatorController.getRightY()));

    // Feed ring out the intake
    operatorController.a().whileTrue(new ParallelCommandGroup(new IntakeRollersManualCommand(intake, () -> -1), new OuttakeRollersManualCommand(outtake, () -> -1)));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    return m_chooser.getSelected();
  }
}

