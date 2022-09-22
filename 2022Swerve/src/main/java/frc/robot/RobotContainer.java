// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveTeleopCommand;
<<<<<<< HEAD
import frc.robot.subsystems.SimulationSubsystem;
=======
>>>>>>> cc1a296ffd5ef6ed657219fb8db009d6b26b76fa
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
<<<<<<< HEAD

  //WHY IS THIS NOT USED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  private final SimulationSubsystem simSub = new SimulationSubsystem(swerveSubsystem);
=======
>>>>>>> cc1a296ffd5ef6ed657219fb8db009d6b26b76fa

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
      swerveSubsystem, 
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
<<<<<<< HEAD
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFODButtonID)));
      
      
    if (simSub == new SimulationSubsystem(swerveSubsystem)) {}

      
=======
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFODButtonID)));

>>>>>>> cc1a296ffd5ef6ed657219fb8db009d6b26b76fa
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
    new JoystickButton(driverJoystick, OIConstants.kDriveZeroButtonID).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
