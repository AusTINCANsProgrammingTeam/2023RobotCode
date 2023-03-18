
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.DebugLog;
import frc.robot.classes.TimeOfFlightSensor;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

  private final double kConeHoldSpeed = 0.35; // change after testing
  private final double kCubeHoldSpeed = 0.25; // change after testing

  private TimeOfFlightSensor timeOfFlightSensor;

  private final double mmConeActivationThreshold = 550.0; 
  private final double mmCubeActivationThreshold = 550.0; 
  private FlightStates tofState = FlightStates.IDLE;

  private CANSparkMax motor;
  private CANSparkMax motor2;
  private static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private static GenericEntry intakeEntry = matchTab.add("Intake Speed", 0.0).getEntry();
  private static GenericEntry intakeMode = matchTab.add("Intake Mode", "Cone Mode").getEntry();

  private DebugLog<Double> coneDist = new DebugLog<Double>(0.0, "Cone Distance", this);
  private DebugLog<Double> cubeDist = new DebugLog<Double>(0.0, "Cube Distance", this);
  private DebugLog<Boolean> hasCone = new DebugLog<Boolean>(false, "Has Cone", this);
  private DebugLog<Boolean> hasCube = new DebugLog<Boolean>(false, "Has Cube", this);
  private DebugLog<Boolean> isConeSensor = new DebugLog<Boolean>(true, "Cone Sensor Working", this);
  private DebugLog<Boolean> isCubeSensor = new DebugLog<Boolean>(true, "Cube Sensor Working", this);

  private boolean isConeMode;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(TimeOfFlightSensor timeOfFlightSensor) {
    isConeMode = true;
    this.timeOfFlightSensor = timeOfFlightSensor;

    motor = MotorController.constructMotor(MotorConfig.IntakeMotor1);
    motor2 = MotorController.constructMotor(MotorConfig.IntakeMotor2);
  }
  
  private void spinWheels(double velocity) {
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

  public void hold() {
    spinWheels(isConeMode ? kConeHoldSpeed : kCubeHoldSpeed);
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

  public void changeFlightState() {
    // Check if sensors are online
    boolean coneSensorUp = timeOfFlightSensor.isSensor0Connected();
    boolean cubeSensorUp = timeOfFlightSensor.isSensor1Connected();

    // Log whether sensors are online
    isConeSensor.log(coneSensorUp);
    isCubeSensor.log(cubeSensorUp);

    // Initialize both values to -1 and only change if sensors are up
    int coneDistance = -1; 
    int cubeDistance = -1;

    // Check if cone sensor is up
    if (coneSensorUp) {
      coneDistance = timeOfFlightSensor.getDistance0();
      hasCone.log(coneDistance <= mmConeActivationThreshold);
    } else {
      hasCone.log(false);
    }

    // Check if cube sensor is up
    if (cubeSensorUp) {
      cubeDistance = timeOfFlightSensor.getDistance1();
      hasCube.log(cubeDistance <= mmCubeActivationThreshold);
    } else {
      hasCube.log(false);
    }

    // Log distance values
    coneDist.log((double)coneDistance);
    cubeDist.log((double)cubeDistance);

    // Change state (only if sensors are online)
    switch(tofState) {
      case IDLE:
        if (coneSensorUp && coneDistance <= mmConeActivationThreshold) {
          tofState = FlightStates.CONE;
        } else if (cubeSensorUp && cubeDistance <= mmCubeActivationThreshold) {
          tofState = FlightStates.CUBE;
        }
      case CONE:
        if (coneSensorUp && coneDistance > mmConeActivationThreshold) {
          tofState = FlightStates.IDLE;
        }
      case CUBE:
        if (cubeSensorUp && cubeDistance > mmCubeActivationThreshold) {
          tofState = FlightStates.IDLE;
        }
      case CONE_SCORE:
        if (coneSensorUp && coneDistance > mmConeActivationThreshold) {
          tofState = FlightStates.IDLE;
        }
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
