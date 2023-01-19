// Copyright (c) FIRST and other WPILib contributors// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;

//import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.SPI;
 
public class AssistedBalance extends SubsystemBase {
 
   boolean LOCKED = false;
   private final SwerveSubsystem swerveSubsystem;
   private final SimulationSubsystem simulationSubsystem;
   //private AHRS gyro = new AHRS(SPI.Port.kMXP);
   private Joystick joystick;
  
   public AssistedBalance(SwerveSubsystem swerveSubsystem, SimulationSubsystem simulationSubsystem) {
       this.swerveSubsystem = swerveSubsystem;
       this.simulationSubsystem = simulationSubsystem;  
       joystick = new Joystick(0);
   }
 
   @Override
   public void periodic() {

    SmartDashboard.putString("|Rotation|", swerveSubsystem.getRotation2d().toString());
    SmartDashboard.putString("Balancing", "No");
    if (joystick.getRawButton(1)) { //Button on keyboard is Z
        SmartDashboard.putString("Balancing", "Yes");
                //if(gyro.isMoving()) {
                    
                    //if (gyro.getPitch() != 0) {
                        
                        if (simulationSubsystem.getPitch() < 360.0 && simulationSubsystem.getPitch() > 180.0) {
                            LOCKED = false;
                            SmartDashboard.putString("Direction", "Forward");
                            swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(1.0, 1.0, 0.0));
                        }  
                        else {
                            swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(-1.0, -1.0, 0.0)); 
                            SmartDashboard.putString("Direction", "Backward");
                        }
                        LOCKED = true;
                        if (LOCKED) {
                            SmartDashboard.putString("BALANCED", "YES");
                            SmartDashboard.putString("Direction", "Off");
                        }
                       
                        else {
                            SmartDashboard.putString("BALANCED", "NO");
                        }
                }
            //}
        //}
    }
}
