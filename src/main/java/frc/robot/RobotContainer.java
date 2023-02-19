// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.classes.Auton;
import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.BuddyBalanceSubsystem;
import frc.robot.commands.AssistedBalanceCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.EverybotIntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final SimulationSubsystem simulationSubsystem;
  private final EverybotIntakeSubsystem intakeSubsystem;
  private final CameraSubsystem cameraSubsystem;
  private final BuddyBalanceSubsystem buddyBalanceSubsystem;

  private Auton auton;

  private AssistedBalanceCommand assistedBalanceCommand;

  private DataLog robotSubsystemsLog = DataLogManager.getLog();
  private StringLogEntry subsystemEnabledLog = new StringLogEntry(robotSubsystemsLog, "/Subsystems Enabled/");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem = Robot.swerveEnabled ? new SwerveSubsystem() : null;
    subsystemEnabledLog.append(swerveSubsystem == null ? "Swerve: Disabled" : "Swerve: Enabled");

    simulationSubsystem = Robot.isSimulation() && Robot.simulationEnabled && swerveSubsystem != null ? new SimulationSubsystem(swerveSubsystem) : null;
    subsystemEnabledLog.append(simulationSubsystem == null ? "Simulation: Disabled" : "Simulation: Enabled");

    intakeSubsystem = Robot.intakeEnabled ? new EverybotIntakeSubsystem() : null;
    subsystemEnabledLog.append(intakeSubsystem == null ? "Intake: Disabled" : "Intake: Enabled");

    cameraSubsystem = Robot.cameraEnabled ? new CameraSubsystem() : null;
    subsystemEnabledLog.append(cameraSubsystem == null ? "Camera: Disabled" : "Camera: Enabled");

    buddyBalanceSubsystem = Robot.buddyBalanceEnabled ? new BuddyBalanceSubsystem() : null;
    subsystemEnabledLog.append(buddyBalanceSubsystem == null ? "Buddy Balance: Disabled" : "Buddy Balance Enabled");

    auton = Robot.swerveEnabled ? new Auton(swerveSubsystem) : null;

    assistedBalanceCommand = Robot.swerveEnabled ? new AssistedBalanceCommand(swerveSubsystem) : null;

    if (Robot.swerveEnabled) {
      swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
      swerveSubsystem, 
      OI.Driver.getXTranslationSupplier(),
      OI.Driver.getYTranslationSupplier(),
      OI.Driver.getRotationSupplier()));
    }



    // Configure the button bindings    
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (Robot.swerveEnabled) {
      OI.Driver.getOrientationButton().onTrue(new InstantCommand(swerveSubsystem::toggleOrientation));
      OI.Driver.getZeroButton().onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
      OI.Driver.getAlignForwardButton().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(0), swerveSubsystem));
      OI.Driver.getAlignBackButton().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(180), swerveSubsystem));
      OI.Driver.getBalanceButton().toggleOnTrue(assistedBalanceCommand); //C on Keyboard
    }
    if (Robot.intakeEnabled) {
    OI.Driver.getIntakeButton().whileTrue(new StartEndCommand(intakeSubsystem::pull, intakeSubsystem::stop, intakeSubsystem));
    OI.Driver.getOuttakeButton().whileTrue(new StartEndCommand(intakeSubsystem::push, intakeSubsystem::stop, intakeSubsystem));
    }

    if (Robot.buddyBalanceEnabled) {
      OI.Operator.getDownBuddyBalanceButton().and(OI.Operator.getActivateBuddyBalanceButton()).onTrue(new InstantCommand(buddyBalanceSubsystem::deployBuddyBalance, buddyBalanceSubsystem).unless(() -> buddyBalanceSubsystem.getIsDeployed()));
      OI.Operator.getDownBuddyBalanceButton().and(OI.Operator.getActivateBuddyBalanceButton()).onTrue(new InstantCommand(buddyBalanceSubsystem::releaseBuddy, buddyBalanceSubsystem).unless(() -> !buddyBalanceSubsystem.getIsDeployed()));
      OI.Operator.getUpBuddyBalanceButton().and(OI.Operator.getActivateBuddyBalanceButton()).onTrue(new InstantCommand(buddyBalanceSubsystem::retrieveBuddy, buddyBalanceSubsystem).unless(() -> !buddyBalanceSubsystem.getIsDeployed()));
    }

    if (!Robot.isCompetition) {
      OI.putControllerButtons();
    }
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auton != null ? auton.getAutonCommand() : null;
  }
}