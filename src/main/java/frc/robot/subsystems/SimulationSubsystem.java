// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.classes.FieldConstants;

public class SimulationSubsystem extends SubsystemBase {

  private final Field2d m_field;
  private final Field2d swerveModuleField;

  private final SwerveSubsystem swerveSubsystem;

  private Translation2d swerveWheelOffset = new Translation2d(8,4);
  private Translation2d[] swerveWheelPos = new Translation2d[4];
  private FieldObject2d[] swerveWheelSim = new FieldObject2d[4];

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
    swerveModuleField = new Field2d();

    for (int i = 0; i < swerveWheelSim.length; i++) {
      swerveWheelSim[i] = swerveModuleField.getObject("SwerveWheel_" + i);
      swerveWheelPos[i] = new Translation2d(Math.pow(-1,i/2) *SwerveSubsystem.kWheelBase/2, Math.pow(-1,i)*SwerveSubsystem.kTrackWidth/2).times(7);
      swerveWheelSim[i].setPose(new Pose2d(swerveWheelPos[i].plus(swerveWheelOffset), new Rotation2d()));
    }

    //This puts the field into SmartDashboard
    SmartDashboard.putData("Field", m_field); 
    SmartDashboard.putData("SwerveModules", swerveModuleField); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    
    //This sets the robot to the correct position on the field.
    m_field.setRobotPose(swerveSubsystem.getPose());

    for (int i = 0; i < swerveWheelSim.length; i++) {
      // Flip angle of direction if wheels are reversed
      Rotation2d rot = swerveSubsystem.getModuleStates()[i].angle.plus(new Rotation2d(simYaw));
      if ( swerveSubsystem.getModuleStates()[i].speedMetersPerSecond < 0) {
        rot = rot.plus(new Rotation2d(Units.degreesToRadians(180)));
      }
      // Update angle of each swerve module
      swerveWheelSim[i].setPose(new Pose2d(swerveWheelPos[i].rotateBy(new Rotation2d(simYaw)).plus(swerveWheelOffset), rot));
    }
    swerveModuleField.setRobotPose(new Pose2d(swerveWheelOffset, m_field.getRobotPose().getRotation()));

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
    angle.set(Units.radiansToDegrees(simYaw));
  }
}