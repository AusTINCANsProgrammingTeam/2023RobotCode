// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SimulationSubsystem extends SubsystemBase {

  private final Field2d m_field;

  private final SwerveSubsystem swerveSubsystem;
  private int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  private double simYaw = 0;
  private double simPitch;
  public double distance = 0;
  SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Pitch"));
  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    m_field = new Field2d(); 

    simPitch = 200;
    
    //This puts the field into SmartDashboard
    SmartDashboard.putData("Field", m_field); 

  }

  public double getPitch() {
    return simPitch;
  }

  public double getDistance() {
    return distance;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    
    //This sets the robot to the correct position on the field.
    m_field.setRobotPose(swerveSubsystem.getPose());

    //Gets the Chassis Speeds (Combined Speeds) from the module states
    ChassisSpeeds chassisSpeed = SwerveSubsystem.kDriveKinematics.toChassisSpeeds(swerveSubsystem.getModuleStates());

    //Assigns the chassisRotationSpeed Value to a variable
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

    //Update simYaw distance
    simYaw += chassisRotationSpeed *  Robot.kDefaultPeriod;

    //Update simPitch
    distance = swerveSubsystem.getVelocity() / 60;
    SmartDashboard.putNumber("a", swerveSubsystem.getVelocity());
    simPitch -= distance;
    SmartDashboard.putNumber("SimPitch", simPitch);

    //Updating Sim NavX
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
    angle.set(-Units.radiansToDegrees(simYaw));
    //pitch.set();
  }
}