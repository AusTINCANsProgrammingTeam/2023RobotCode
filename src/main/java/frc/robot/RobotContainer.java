// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.AutonSubsytem;
import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.BatterySubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.OI.Driver;
import frc.robot.commands.ArmCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final EverybotIntakeSubsystem intakeSubsystem = new EverybotIntakeSubsystem();

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final AutonSubsytem autonSubsytem = new AutonSubsytem(swerveSubsystem);
  private SimulationSubsystem simulationSubsystem;
  
  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();

  private static BatterySubsystem batterySubsystem;
  //TODO: Get a joystick
  private final ArmCommand armCommand = new ArmCommand(armSubsystem, OI.Operator.getArmRotationSupplier());
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(Robot.isSimulation()){
      simulationSubsystem = new SimulationSubsystem(swerveSubsystem);
    }
    if (!Robot.isCompetition) {
      batterySubsystem = new BatterySubsystem();
    }
    swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
      swerveSubsystem, 
      OI.Driver.getXTranslationSupplier(),
      OI.Driver.getYTranslationSupplier(),
      OI.Driver.getRotationSupplier()));

    armSubsystem.setDefaultCommand(armCommand);
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
    OI.Driver.getOrientationButton().onTrue(new InstantCommand(swerveSubsystem::toggleOrientation));
    OI.Driver.getArmButton().onTrue(new InstantCommand(armSubsystem::toggleArmControl));
    OI.Driver.getZeroButton().onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    OI.Driver.getAlignForwardPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(0), swerveSubsystem));
    OI.Driver.getAlignBackPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(180), swerveSubsystem));
    OI.Driver.getAlignLeftPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(90), swerveSubsystem));
    OI.Driver.getAlignRightPOV().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(-90), swerveSubsystem));
    OI.Driver.getIntakeButton().onTrue(new InstantCommand(intakeSubsystem::pull, intakeSubsystem));
    OI.Driver.getOuttakeButton().onTrue(new InstantCommand(intakeSubsystem::push, intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonSubsytem.getAutonCommand();
  }
}
