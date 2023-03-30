// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.TunableNumber;
import frc.robot.commands.AssistedBalanceCommand;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController.MotorConfig;

public class SwerveSubsystem extends SubsystemBase{
    public static final double kPhysicalMaxSpeed = Units.feetToMeters(14.5);; //Max drivebase speed in meters per second
    public static final double kPhysicalMaxAngularSpeed = 2 * Math.PI; //Max drivebase angular speed in radians per second

    public static final double kTrackWidth = Units.inchesToMeters(19.75); //Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(19.75); //Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( //Creates robot geometry using the locations of the 4 wheels
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
        new Translation2d(kWheelBase / 2, -kTrackWidth /2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        
    public static final double kXTranslationP = 1.75;
    public static final double kYTranslationP = 1.75;
    public static final double kRotationP = 1.75;
    public static final double kRotationI = 1e-6;

    public static final double kAutoRotationP = 0.575;

    private final SwerveModule frontLeft = new SwerveModule(
        MotorConfig.FrontLeftModuleDrive,
        MotorConfig.FrontLeftModuleTurn,
        EncoderConfig.FrontLeftModule,
        "FL");

    private final SwerveModule frontRight = new SwerveModule(
        MotorConfig.FrontRightModuleDrive,
        MotorConfig.FrontRightModuleTurn,
        EncoderConfig.FrontRightModule,
        "FR");

    private final SwerveModule backLeft = new SwerveModule(
        MotorConfig.BackLeftModuleDrive,
        MotorConfig.BackLeftModuleTurn,
        EncoderConfig.BackLeftModule,
        "BL");

    private final SwerveModule backRight = new SwerveModule(
        MotorConfig.BackRightModuleDrive,
        MotorConfig.BackRightModuleTurn,
        EncoderConfig.BackRightModule,
        "BR");

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private double gyroOffset; //Offset in degrees
    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(kDriveKinematics, getRotation2d(), getModulePositions());

    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry translationXOutputLog = new DoubleLogEntry(datalog, "/swerve/txout"); //Logs x translation state output
    private DoubleLogEntry translationYOutputLog = new DoubleLogEntry(datalog, "/swerve/tyout"); //Logs y translation state output
    private DoubleLogEntry rotationOutputLog = new DoubleLogEntry(datalog, "/swerve/rotout"); //Logs rotation state output
    private BooleanLogEntry controlOrientationLog = new BooleanLogEntry(datalog, "/swerve/orientation"); //Logs if robot is in FOD/ROD
    private StringLogEntry errors = new StringLogEntry(datalog, "/swerve/errors"); //Logs any hardware errors
    private StringLogEntry trajectoryLog = new StringLogEntry(datalog, "/auton/trajectory"); //Logs autonomous trajectory following

    private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    private GenericEntry controlOrientationEntry = matchTab.add("FOD", true).getEntry();
    private GenericEntry headingEntry = matchTab.add("NavX Yaw", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    private GenericEntry pitchEntry = matchTab.add("NavX Pitch", 0).withWidget(BuiltInWidgets.kGyro).getEntry();

    private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
    private GenericEntry positionEntry = configTab.add("Position", "").getEntry();

    public boolean controlOrientationIsFOD;

    public Double rotationHold;

    private PIDController xController;
    private PIDController yController;
    private PIDController rotationController;
    private PIDController autoRotationController;

    private TunableNumber translationXTuner;
    private TunableNumber translationYTuner;
    private TunableNumber rotationPTuner;
    private TunableNumber rotationITuner;

    public SwerveSubsystem() {
        zeroHeading();
        controlOrientationIsFOD = true;

        //Add coast mode command to shuffleboard
        configTab.add(new StartEndCommand(this::coastModules, this::brakeModules, this).ignoringDisable(true).withName("Coast Modules"));

        //Define PID controllers for tracking trajectory
        xController = new PIDController(kXTranslationP, 0, 1e-4);
        yController = new PIDController(kYTranslationP, 0, 1e-4);
        rotationController = new PIDController(kRotationP, kRotationI, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        autoRotationController = new PIDController(kAutoRotationP, 0, 0);
        autoRotationController.enableContinuousInput(-Math.PI, Math.PI);

        translationXTuner = new TunableNumber("X Translation P", kXTranslationP, xController::setP);
        translationYTuner = new TunableNumber("Y Translation P", kYTranslationP, yController::setP);
        rotationPTuner = new TunableNumber("Rotation P", kRotationP, rotationController::setP);
        rotationITuner = new TunableNumber("Rotation I", kRotationI, rotationController::setI);
    }

    public void zeroHeading() {
        if (gyro.isCalibrating()){errors.append("gyro failed to calibrate before zero");} 
        gyro.reset();
        gyroOffset = 0;
    }

    public void zeroHeading(Rotation2d rotation2d) {
        if (gyro.isCalibrating()){errors.append("gyro failed to calibrate before zero");} 
        gyro.reset();
        gyroOffset = rotation2d.getDegrees();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle() + gyroOffset, 360);
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void toggleOrientation(){
        //Toggle control orientation from FOD/ROD
        controlOrientationIsFOD = !controlOrientationIsFOD;
        controlOrientationLog.append(controlOrientationIsFOD);
        controlOrientationEntry.setBoolean(controlOrientationIsFOD);
    }

    public void enableRotationHold(int angle){
        //Set the angle to automatically align the drive to using degrees -180 to 180
        rotationHold = Units.degreesToRadians(angle);
    }

    public void disableRotationHold(){
        rotationHold = null;
    }

    public SwerveModuleState[] convertToModuleStates(double xTranslation, double yTranslation, double rotation) {
        //Takes axis input from joysticks and returns an array of swerve module states

        double x = yTranslation; //Intentional, x in swerve kinematics is y on the joystick
        double y = xTranslation;
        double r = rotation;

        if (Math.abs(r) > 0){
            disableRotationHold();
        }
        else if(rotationHold != null){
            r = autoRotationController.calculate(Units.degreesToRadians(getHeading()), rotationHold);
        }

        //Map to speeds in meters/radians per second
        x *= kPhysicalMaxSpeed;
        y *= kPhysicalMaxSpeed;
        r *= kPhysicalMaxAngularSpeed;

        //Log speeds used to construct chassis speeds
        translationXOutputLog.append(x);
        translationYOutputLog.append(y);
        rotationOutputLog.append(r);

        //Construct Chassis Speeds
        ChassisSpeeds chassisSpeeds;
        if(controlOrientationIsFOD){
            //Field Oriented Drive
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, this.getRotation2d());
        } else {
            //Robot Oriented Drive
            chassisSpeeds = new ChassisSpeeds(x, y, r);
        }
        SmartDashboard.putString("chassis speeds",chassisSpeeds.toString());
        //Convert Chassis Speeds to individual module states
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        Logger.getInstance().recordOutput("Desired States", moduleStates);
        return moduleStates;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveModuleArray = new SwerveModuleState[4];
        swerveModuleArray[0] = frontLeft.getState();
        swerveModuleArray[1] = frontRight.getState();
        swerveModuleArray[2] = backLeft.getState();
        swerveModuleArray[3] = backRight.getState();

        return swerveModuleArray;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swerveModuleArray = new SwerveModulePosition[4];
        swerveModuleArray[0] = frontLeft.getPosition();
        swerveModuleArray[1] = frontRight.getPosition();
        swerveModuleArray[2] = backLeft.getPosition();
        swerveModuleArray[3] = backRight.getPosition();

        return swerveModuleArray;
    }  

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void parkModules(){
        //this tells the method in swerve module which wheels should be 45 degrees and which ones should be -45 degrees
        frontLeft.park(true);
        frontRight.park(false);
        backLeft.park(false);
        backRight.park(true);
    }

    public void coastModules(){
        frontLeft.coast();
        frontRight.coast();
        backLeft.coast();
        backRight.coast();
    }

    public void brakeModules(){
        frontLeft.brake();
        frontRight.brake();
        backLeft.brake();
        backRight.brake();
    }
    
    public Command followTrajectory(String name, PathPlannerTrajectory trajectory){
        //For use with trajectories generated from a list of poses
        return new PPSwerveControllerCommand(
            trajectory,
            this::getPose, 
            SwerveSubsystem.kDriveKinematics, 
            xController, 
            yController, 
            rotationController, 
            this::setModuleStates, 
            true,
            this
        ).beforeStarting(() -> trajectoryLog.append("Following trajectory " + name)
        ).alongWith(new InstantCommand(() -> Logger.getInstance().recordOutput("trajectory " + name, trajectory)));
    }

    public Command assistedBalance(boolean reversed){
        return new AssistedBalanceCommand(this, reversed);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putData(this);
        odometer.update(getRotation2d(), getModulePositions());
        pitchEntry.setDouble(getPitch());
        headingEntry.setDouble(getHeading());
        positionEntry.setString(getPose().getTranslation().toString());
        Logger.getInstance().recordOutput("Actual Module States", getModuleStates());
        Logger.getInstance().recordOutput("Pose 2D", getPose());
    }
}