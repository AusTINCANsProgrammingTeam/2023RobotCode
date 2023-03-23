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
import frc.robot.Robot.LedEnum;
import frc.robot.classes.Auton;
import frc.robot.classes.TimeOfFlightSensor;
import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.commands.ArmAnglesCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.BuddyBalanceSubsystem;
import frc.robot.subsystems.led.BlinkinLedSubsystem;
import frc.robot.subsystems.led.LedMatrixSubsystem;
import frc.robot.subsystems.led.LedStripSubsystem;
import frc.robot.subsystems.led.BlinkinLedSubsystem.BlinkinMode;
import frc.robot.subsystems.led.LedMatrixSubsystem.MatrixMode;
import frc.robot.subsystems.led.LedStripSubsystem.StripMode;
import frc.robot.commands.ToFIntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CubeapultSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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
  private final IntakeSubsystem intakeSubsystem;
  private final CameraSubsystem cameraSubsystem;
  private final BuddyBalanceSubsystem buddyBalanceSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ArmAnglesCommand armAnglesCommand;
  private final CubeapultSubsystem cubeapultSubsystem;

  private LedStripSubsystem ledSubsystem;
  private LedMatrixSubsystem ledMatrixSubsystem;
  private BlinkinLedSubsystem blinkinLedSubsystem;
  
  private Auton auton;
  private TimeOfFlightSensor timeOfFlightSensor;

  private DataLog robotSubsystemsLog = DataLogManager.getLog();
  private StringLogEntry subsystemEnabledLog = new StringLogEntry(robotSubsystemsLog, "/Subsystems Enabled/"); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSubsystem = Robot.armEnabled ? new ArmSubsystem() : null;
    subsystemEnabledLog.append(armSubsystem == null ? "Arm: Disabled" : "Arm: Enabled");

    ledSubsystem = Robot.ledSubSelect == LedEnum.STRIP ? new LedStripSubsystem() : null;
    subsystemEnabledLog.append(ledSubsystem == null ? "Led: Disabled" : "Led: Enabled");

    ledMatrixSubsystem = Robot.ledSubSelect == LedEnum.MATRIX ? new LedMatrixSubsystem() : null; //PWM port 2
    subsystemEnabledLog.append(ledSubsystem == null ? "Led Matrix: Disabled" : "Led Matrix: Enabled");

    blinkinLedSubsystem = Robot.ledSubSelect == LedEnum.BLINKIN ? new BlinkinLedSubsystem() : null;
    subsystemEnabledLog.append(ledSubsystem == null ? "Led Blinkin: Disabled" : "Led Blinkin: Enabled");

    swerveSubsystem = Robot.swerveEnabled ? new SwerveSubsystem() : null;
    subsystemEnabledLog.append(swerveSubsystem == null ? "Swerve: Disabled" : "Swerve: Enabled");

    simulationSubsystem = Robot.isSimulation() && swerveSubsystem != null ? new SimulationSubsystem(swerveSubsystem) : null;
    subsystemEnabledLog.append(simulationSubsystem == null ? "Simulation: Disabled" : "Simulation: Enabled");

    if (Robot.intakeEnabled) {
      if (Robot.tofEnabled) {
        timeOfFlightSensor = new TimeOfFlightSensor();
        intakeSubsystem = new IntakeSubsystem(timeOfFlightSensor);
      } else {
        intakeSubsystem = new IntakeSubsystem();
      }
    } else {
      intakeSubsystem = null;
    }

    subsystemEnabledLog.append(intakeSubsystem == null ? "Intake: Disabled" : "Intake: Enabled");

    cameraSubsystem = Robot.cameraEnabled ? new CameraSubsystem() : null;
    subsystemEnabledLog.append(cameraSubsystem == null ? "Camera: Disabled" : "Camera: Enabled");

    buddyBalanceSubsystem = Robot.buddyBalanceEnabled ? new BuddyBalanceSubsystem() : null;
    subsystemEnabledLog.append(buddyBalanceSubsystem == null ? "Buddy Balance: Disabled" : "Buddy Balance: Enabled");

    cubeapultSubsystem = Robot.cubeapultEnabled ? new CubeapultSubsystem() : null;
    subsystemEnabledLog.append(cubeapultSubsystem == null ? "Cubeapult: Disabled" : "Cubeapult: Enabled");

    auton = Robot.swerveEnabled ? new Auton(swerveSubsystem, armSubsystem, intakeSubsystem, cubeapultSubsystem) : null;

    armAnglesCommand = Robot.armEnabled ? new ArmAnglesCommand(armSubsystem, OI.Operator.getArmBaseSupplier(), OI.Operator.getArmElbowSupplier()) : null;

    if (Robot.swerveEnabled) {
      swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
      swerveSubsystem, 
      OI.Driver.getXTranslationSupplier(),
      OI.Driver.getYTranslationSupplier(),
      OI.Driver.getRotationSupplier()));
    }

    if (Robot.armEnabled) {
      armSubsystem.setDefaultCommand(armAnglesCommand);
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
    if (Robot.ledSubSelect == LedEnum.MATRIX){
     // OI.Operator.getYellowCargoButton().onTrue(ledMatrixSubsystem.goCans());
     OI.Operator.getConeSignalButton().whileTrue(new StartEndCommand(() -> ledMatrixSubsystem.cargoLed(MatrixMode.CONE), ledMatrixSubsystem::offLed, ledMatrixSubsystem).withName("Cone Signal"));
     OI.Operator.getCubeSignalButton().whileTrue(new StartEndCommand(() -> ledMatrixSubsystem.cargoLed(MatrixMode.CUBE), ledMatrixSubsystem::offLed, ledMatrixSubsystem).withName("Cube Signal"));
    }
    if (Robot.ledSubSelect == LedEnum.BLINKIN){
      OI.Operator.getConeSignalButton().whileTrue(new StartEndCommand(() -> blinkinLedSubsystem.cargoLed(BlinkinMode.BLINKIN_YELLOW), blinkinLedSubsystem::blinkinStopLed, blinkinLedSubsystem).withName("Cone Signal"));
      OI.Operator.getCubeSignalButton().whileTrue(new StartEndCommand(() -> blinkinLedSubsystem.cargoLed(BlinkinMode.BLINKIN_PURPLE), blinkinLedSubsystem::blinkinStopLed, blinkinLedSubsystem).withName("Cube Signal"));
    }
    if (Robot.ledSubSelect == LedEnum.STRIP){
      OI.Operator.getConeSignalButton().whileTrue(new StartEndCommand(() -> ledSubsystem.cargoLed(StripMode.CUBE), ledSubsystem::offLed, ledSubsystem).withName("Cone Signal"));
      OI.Operator.getCubeSignalButton().whileTrue(new StartEndCommand(() -> ledSubsystem.cargoLed(StripMode.CONE), ledSubsystem::offLed, ledSubsystem).withName("Cube Signal"));
    }
    if (Robot.swerveEnabled) {
      OI.Driver.getOrientationButton().onTrue(new InstantCommand(swerveSubsystem::toggleOrientation));
      OI.Driver.getZeroButton().onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
      OI.Driver.getAlignForwardButton().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(0), swerveSubsystem));
      OI.Driver.getAlignBackButton().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(180), swerveSubsystem));
    }

    if (Robot.intakeEnabled) {
      OI.Driver.getIntakeButton().whileTrue(new StartEndCommand(intakeSubsystem::pull, intakeSubsystem::stop, intakeSubsystem).withName("Intake"));
      OI.Driver.getOuttakeButton().whileTrue(new StartEndCommand(intakeSubsystem::push, intakeSubsystem::stop, intakeSubsystem).withName("Outtake"));
      OI.Operator.getIntakeButton().onTrue(new InstantCommand(intakeSubsystem::toggleConeMode));
    }

    if (Robot.armEnabled) {
      OI.Driver.getArmHighButton().onTrue(new ProxyCommand(armSubsystem::handleHighButton));
      OI.Driver.getArmMidButton().onTrue(new ProxyCommand(armSubsystem::handleMidButton));
      OI.Operator.getArmStopButton().onTrue(new InstantCommand(armSubsystem::cancelCommands));
      if (Robot.intakeEnabled){
        OI.Driver.getArmConeIntakeButton().onTrue(new ProxyCommand(() -> armSubsystem.handleConeIntakeButton()).alongWith(new InstantCommand(() -> intakeSubsystem.setConeMode())));
        OI.Driver.getArmCubeIntakeButton().onTrue(new ProxyCommand(() -> armSubsystem.handleCubeIntakeButton()).alongWith(new InstantCommand(() -> intakeSubsystem.setCubeMode())));
        if (Robot.tofEnabled) {
          intakeSubsystem.setDefaultCommand(new ToFIntakeCommand(intakeSubsystem, armSubsystem));
        }
      }
      OI.Operator.getHighScoreButton().onTrue(armSubsystem.goToState(ArmState.HIGHSCORE));
    }

    if (Robot.buddyBalanceEnabled) {
      OI.Operator.getDownBuddyBalanceButton().and(OI.Operator.getActivateBuddyBalanceButton()).onTrue(new InstantCommand(buddyBalanceSubsystem::deployBuddyBalance, buddyBalanceSubsystem).unless(() -> buddyBalanceSubsystem.getIsDeployed()));
      OI.Operator.getDownBuddyBalanceButton().and(OI.Operator.getActivateBuddyBalanceButton()).onTrue(new InstantCommand(buddyBalanceSubsystem::releaseBuddy, buddyBalanceSubsystem).unless(() -> !buddyBalanceSubsystem.getIsDeployed()));
      OI.Operator.getUpBuddyBalanceButton().and(OI.Operator.getActivateBuddyBalanceButton()).onTrue(new InstantCommand(buddyBalanceSubsystem::retrieveBuddy, buddyBalanceSubsystem).unless(() -> !buddyBalanceSubsystem.getIsDeployed()));
    }

    OI.putControllerButtons();
    
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
