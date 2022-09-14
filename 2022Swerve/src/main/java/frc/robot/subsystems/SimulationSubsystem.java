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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SimulationSubsystem extends SubsystemBase {

  private final Field2d m_field;
  
  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;
  
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;
  private int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  private double simYaw = 0;
  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    m_frontLeftLocation = new Translation2d(0.381, 0.381);
    m_frontRightLocation = new Translation2d(0.381, -0.381);
    m_backLeftLocation = new Translation2d(-0.381, 0.381);
    m_backRightLocation = new Translation2d(-0.381, -0.381);

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    m_odometry = new SwerveDriveOdometry(m_kinematics, swerveSubsystem.getRotation2d(), new Pose2d(0, 0, new Rotation2d()));

    m_field = new Field2d(); 
    SmartDashboard.putData("Field", m_field); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run

    m_odometry.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModuleStates());
    //ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(swerveSubsystem.getModuleStates()); 


    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose Rotation", m_odometry.getPoseMeters().getRotation().getDegrees());

    var chassisSpeed = m_kinematics.toChassisSpeeds(swerveSubsystem.getModuleStates());
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

    simYaw += chassisRotationSpeed * 0.02;
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
    angle.set(-Units.radiansToDegrees(simYaw));
  }
}
