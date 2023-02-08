// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.classes.FieldConstants;

public class SimulationSubsystem extends SubsystemBase {

  private final Field2d m_field;

  private final SwerveSubsystem swerveSubsystem;
  private int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  private double simYaw = 0;
  private double simPitch;
  private double swerveX;
  private SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Pitch"));
  private SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    m_field = new Field2d(); 
    
    //This puts the field into SmartDashboard
    SmartDashboard.putData("Field", m_field); 

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
    swerveX = swerveSubsystem.getPose().getX();
    
    if (swerveX >= 0 && 
        swerveX <= FieldConstants.Community.chargingStationLength) {
      simPitch = Units.degreesToRadians((swerveX * ((FieldConstants.Community.chargingStationAngle * 2) / FieldConstants.Community.chargingStationLength)) - FieldConstants.Community.chargingStationAngle);
    }
    else {
      simPitch = 0;
    }

    pitch.set(simPitch);

    //Updating Sim NavX
    angle.set(-Units.radiansToDegrees(simYaw));
  }
}