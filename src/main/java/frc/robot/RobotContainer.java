// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.classes.Auton;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.BatterySubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.EverybotIntakeSubsystem;
import frc.robot.classes.Auton;

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
  private final EverybotIntakeSubsystem everybotIntakeSubsystem;
  private final CameraSubsystem cameraSubsystem;
  private static BatterySubsystem batterySubsystem;

  private Auton auton;

  private DataLog robotSubsystemsLog = DataLogManager.getLog();
  private StringLogEntry subsystemEnabledLog = new StringLogEntry(robotSubsystemsLog, "/Subsystems Enabled/");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem = Robot.swerveEnabled ? new SwerveSubsystem() : null;
    subsystemEnabledLog.append(swerveSubsystem == null ? "Swerve: Disabled" : "Swerve: Enabled");

    simulationSubsystem = Robot.isSimulation() ? new SimulationSubsystem(swerveSubsystem) : null;
    subsystemEnabledLog.append(simulationSubsystem == null ? "Simulation: Disabled" : "Simulation: Enabled");

    everybotIntakeSubsystem = Robot.intakeEnabled ? new EverybotIntakeSubsystem() : null;
    subsystemEnabledLog.append(everybotIntakeSubsystem == null ? "Intake: Disabled" : "Intake: Enabled");

    cameraSubsystem = Robot.cameraEnabled ? new CameraSubsystem() : null;
    subsystemEnabledLog.append(cameraSubsystem == null ? "Camera: Disabled" : "Camera: Enabled");

    batterySubsystem = Robot.batteryEnabled && !Robot.isCompetition ? new BatterySubsystem() : null;
    subsystemEnabledLog.append(batterySubsystem == null ? "Battery: Disabled" : "Battery: Enabled");

    auton = Robot.swerveEnabled ? new Auton(swerveSubsystem) : null;

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
      OI.Driver.getAlignForwardPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(0), swerveSubsystem));
      OI.Driver.getAlignBackPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(180), swerveSubsystem));
      OI.Driver.getAlignLeftPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(90), swerveSubsystem));
      OI.Driver.getAlignRightPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(-90), swerveSubsystem));
    }
    if (Robot.intakeEnabled) {
      OI.Driver.getIntakeButton().onTrue(new InstantCommand(everybotIntakeSubsystem::pull, everybotIntakeSubsystem));
      OI.Driver.getOuttakeButton().onTrue(new InstantCommand(everybotIntakeSubsystem::push, everybotIntakeSubsystem));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auton.getAutonCommand();
  }
}
