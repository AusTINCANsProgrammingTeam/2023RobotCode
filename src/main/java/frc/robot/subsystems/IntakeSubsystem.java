
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
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
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
  public static final double kCubeOuttakeSpeed = -0.65;

  private final double kConeHoldSpeed = 0.35; // change after testing
  private final double kCubeHoldSpeed = 0.25; // change after testing

  private TimeOfFlightSensor timeOfFlightSensor;

  private final double mmConeActivationThreshold = 450.0; 
  private final double mmCubeActivationThreshold = 450.0; 
  private FlightStates tofState = FlightStates.IDLE;

  private CANSparkMax motor;
  private CANSparkMax motor2;

  private static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private static GenericEntry intakeEntry = matchTab.add("Intake Speed", 0.0).getEntry();
  private static GenericEntry intakeMode = matchTab.add("Intake Mode", true).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("colorWhenFalse", "Purple", "colorWhenTrue", "Yellow")).getEntry();
  private static GenericEntry currentState = matchTab.add("ToF State", FlightStates.IDLE.toString()).getEntry();
  private static GenericEntry sensor0Up = matchTab.add("Cone ToF sensor up: ", true).getEntry();
  private static GenericEntry sensor1Up = matchTab.add("Cube ToF sensor up: ", true).getEntry(); 

  private DebugLog<Double> coneDistLog = new DebugLog<Double>(0.0, "Cone Distance", this);
  private DebugLog<Double> cubeDistLog = new DebugLog<Double>(0.0, "Cube Distance", this);
  private DebugLog<Boolean> hasConeLog = new DebugLog<Boolean>(false, "Has Cone", this);
  private DebugLog<Boolean> hasCubeLog = new DebugLog<Boolean>(false, "Has Cube", this);

  private boolean isConeMode;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    isConeMode = true;

    motor = MotorController.constructMotor(MotorConfig.IntakeMotor1);
    motor2 = MotorController.constructMotor(MotorConfig.IntakeMotor2);
  }

  public IntakeSubsystem(TimeOfFlightSensor timeOfFlightSensor) {
    this();
    this.timeOfFlightSensor = timeOfFlightSensor;
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

    public void setConeMode() {
      this.isConeMode = true;
  }

    public void setCubeMode() {
      this.isConeMode = false;
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

  public void hold() {
    spinWheels(isConeMode ? kConeIntakeSpeed : kCubeIntakeSpeed);
  }

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motor.close();
    motor2.close();
  }

  public void changeFlightState() {
    // Check distances
    int coneDistance = timeOfFlightSensor.getDistance0();
    int cubeDistance = timeOfFlightSensor.getDistance1();

    // Log distance values
    coneDistLog.log((double)coneDistance);
    cubeDistLog.log((double)cubeDistance);

    // Check if sensors are online
    boolean coneSensorUp = coneDistance != -1;
    boolean cubeSensorUp = cubeDistance != -1;

    // Log whether sensors are online
    sensor0Up.setBoolean(coneSensorUp);
    sensor1Up.setBoolean(cubeSensorUp);

    // Log if sensors are activated
    hasConeLog.log(coneDistance > 0 && coneDistance <=mmConeActivationThreshold);
    hasCubeLog.log(cubeDistance > 0 && cubeDistance <=mmCubeActivationThreshold);

    // Change state (only if sensors are online)
    switch(tofState) {
      case IDLE:
        if (coneSensorUp && coneDistance <= mmConeActivationThreshold) {
          tofState = FlightStates.CONE;
          break;
        } else if (cubeSensorUp && cubeDistance <= mmCubeActivationThreshold) {
          tofState = FlightStates.CUBE;
          break;
        }
      case CONE:
        if (coneSensorUp && coneDistance > mmConeActivationThreshold) {
          tofState = FlightStates.IDLE;
          break;
        }
      case CUBE:
        if (cubeSensorUp && cubeDistance > mmCubeActivationThreshold) {
          tofState = FlightStates.IDLE;
          break;
        }
      case CONE_SCORE:
        if (coneSensorUp && coneDistance > mmConeActivationThreshold) {
          tofState = FlightStates.IDLE;
          break;
        }
    }
  }

  public FlightStates getFlightState() {
    // Check if state has changed before returning it
    changeFlightState();
    currentState.setString(tofState.toString());
    return tofState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
    intakeMode.setString((isConeMode) ? "Cone Mode" : "Cube Mode");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
