
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.TimeOfFlightSensor;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public enum FlightStates{
    IDLE,
    CUBE,
    CONE,
    CONE_SCORE
  }

  public static final double kConeIntakeSpeed = -0.75;
  public static final double kConeOuttakeSpeed = 0.75;
  public static final double kCubeIntakeSpeed = 0.55;
  public static final double kCubeOuttakeSpeed = -0.55;
  private TimeOfFlightSensor timeOfFlightSensor;

  private final double coneActivationThreshold = 25.0; // placeholder value for how small values have to be for cone to be there
  private final double cubeActivationThreshold = 25.0; // placeholder value for how small values have to be for cube to be there
  private FlightStates tofState = FlightStates.IDLE;

  private CANSparkMax motor;
  private CANSparkMax motor2;
  private static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private static GenericEntry intakeEntry = matchTab.add("Intake Speed", 0.0).getEntry();
  private static GenericEntry intakeMode = matchTab.add("Intake Mode", "Cone Mode").getEntry();
  private boolean isConeMode;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(TimeOfFlightSensor timeOfFlightSensor) {
    isConeMode = true;
    this.timeOfFlightSensor = timeOfFlightSensor;

    motor = MotorController.constructMotor(MotorConfig.IntakeMotor1);
    motor2 = MotorController.constructMotor(MotorConfig.IntakeMotor2);
  }
  
  public void spinWheels(double velocity) {
    motor.set(velocity);
    motor2.set(-velocity);
    intakeEntry.setDouble(velocity);
  }

  public void push() {
    spinWheels(isConeMode ? kConeOuttakeSpeed : kCubeOuttakeSpeed);
  }

  public void pull() {
    spinWheels(isConeMode ? kConeIntakeSpeed : kCubeIntakeSpeed);
  }

  public void setMode(boolean isConeMode) {
    intakeMode.setString((isConeMode) ? "Cone Mode" : "Cube Mode");
    this.isConeMode = isConeMode;
  }

  public void toggleConeMode() {
    isConeMode = !isConeMode;
  }
  
  public double getSpeed(){
    return motor.get();
  }

  public void stop() {
    spinWheels(0);
  }

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motor.close();
    motor2.close();
  }

  public double getConeDist() {
    return timeOfFlightSensor.getRawDistance0().classDistance;
  }

  public double getCubeDist() {
    return timeOfFlightSensor.getRawDistance1().classDistance;
  }

  public void changeFlightState() {
    double coneDistance = getConeDist();
    double cubeDistance = getCubeDist();

    boolean coneSensorUp = coneDistance != -1;
    boolean cubeSensorUp = cubeDistance != -1;

    SmartDashboard.putNumber("Cone Distance", coneDistance);
    SmartDashboard.putNumber("Cube Distance", cubeDistance);
    SmartDashboard.putBoolean("Has Cone", coneDistance <= coneActivationThreshold);
    SmartDashboard.putBoolean("Has Cube", cubeDistance <= cubeActivationThreshold);
    SmartDashboard.putBoolean("Cone Sensor Working", coneSensorUp);
    SmartDashboard.putBoolean("Cube Sensor Working", cubeSensorUp);

    // Only change state if both sensors are up, otherwise stay in last state
    if (coneDistance <= coneActivationThreshold && coneSensorUp) {
      tofState = FlightStates.CONE;
    } 
    else if (cubeDistance <= cubeActivationThreshold && cubeSensorUp) {
      tofState = FlightStates.CUBE;
    } 
    else {
      tofState = FlightStates.IDLE;
    }
  }

  public FlightStates getFlightState() {
    // Check if state has changed before returning it
    changeFlightState();
    return tofState;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
