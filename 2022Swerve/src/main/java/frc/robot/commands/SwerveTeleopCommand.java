package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turnSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction; //Whether or not control is field oriented
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> turnSpeedFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turnSpeedFunction = turnSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
        this.turnLimiter = new SlewRateLimiter(DriveConstants.kMaxAngluarAcceleration);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //Get joystick inputs
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double turnSpeed = turnSpeedFunction.get();

        //Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;

        //Smooth input using a slew limiter, map to a speeds in meters/radians per second
        xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kPhysicalMaxSpeed / DriveConstants.kSpeedFactor);
        ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kPhysicalMaxSpeed / DriveConstants.kSpeedFactor);
        turnSpeed = turnLimiter.calculate(turnSpeed) * (DriveConstants.kPhysicalMaxAngularSpeed / DriveConstants.kAngularSpeedFactor);

        //Construct Chassis Speeds
        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()){
            //Field Oriented Drive
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        } else {
            //Robot Oriented Drive
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }

        //Convert Chassis Speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
