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

public class SimulationSubsystem extends SubsystemBase {

  private final Field2d m_field;

  private final SwerveSubsystem swerveSubsystem;
  private int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  private double simYaw = 0;
  private double simPitch;
  private double swerveX;
  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    m_field = new Field2d(); 

    simPitch = 100;
    
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
    
    if (swerveX >= 0 && swerveX <= Units.feetToMeters(4)) {
      simPitch = Units.degreesToRadians((swerveX * (22/Units.feetToMeters(4))) - 11);
    }
    else {
      simPitch = 0;
    }

    SmartDashboard.putNumber("PoseX", swerveX);
    SmartDashboard.putNumber("SimPitch", simPitch);

    SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Pitch"));
    pitch.set(simPitch);
    
    SmartDashboard.putNumber("GyroPitch", swerveSubsystem.getPitch());


    //Updating Sim NavX
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
    angle.set(-Units.radiansToDegrees(simYaw));
  }
}