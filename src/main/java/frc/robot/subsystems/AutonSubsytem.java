package frc.robot.subsystems;

import java.util.Objects;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;


public class AutonSubsytem extends SubsystemBase{
    private enum AutonModes{
        FORWARD, // Go forward 2 meters
        BACKWARD, // Wait 3 seconds, go backward 2 meters
        FORWARD180; // Go forward 2 meters and rotate 180 degrees
    }
    private final AutonModes kDefaultAutonMode = AutonModes.FORWARD;

    private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
    private NetworkTableEntry delayEntry = configTab.add("Auton Delay", 0.0).getEntry();
    private SendableChooser<AutonModes> modeChooser = new SendableChooser<>();

    private DataLog datalog = DataLogManager.getLog();
    private StringLogEntry trajectoryLog = new StringLogEntry(datalog, "/auton/trajectory"); //Logs x translation state output
    private StringLogEntry commandLog = new StringLogEntry(datalog, "/auton/command"); //Logs x translation state output

    private SwerveSubsystem swerveSubsystem;

    private PIDController xController;
    private PIDController yController;
    private PIDController rotationController;

    private PathConstraints pathConstraints;

    private AutonModes autonMode;

    public AutonSubsytem(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        //Add auton modes to chooser
        for(AutonModes mode : AutonModes.values()){
            modeChooser.addOption(mode.toString(), mode);
        }
        modeChooser.setDefaultOption(kDefaultAutonMode.toString(), kDefaultAutonMode);
        configTab.add("Auton mode", modeChooser);

        //Define PID controllers for tracking trajectory
        xController = new PIDController(AutonConstants.kXTranslationP, 0, 0);
        yController = new PIDController(AutonConstants.kYTranslationP, 0, 0);
        rotationController = new PIDController(AutonConstants.kRotationP, 0, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        pathConstraints = new PathConstraints(AutonConstants.kMaxSpeed, AutonConstants.kMaxAcceleration);
    }

    private PathPlannerTrajectory getTrajectory(String name) throws NullPointerException{
        return PathPlanner.loadPath(name, pathConstraints);
    }

    private PathPoint constructPoint(double x, double y, double rotation, double heading){
        return new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(heading), Rotation2d.fromDegrees(rotation));
    }

    private PathPlannerTrajectory generateTrajectory(PathPoint firstPoint, PathPoint secondPoint, PathPoint... points){
        return PathPlanner.generatePath(
            pathConstraints,
            firstPoint,
            secondPoint,
            points
        );
    }

    private Command followTrajectory(String name) throws NullPointerException{
        //For use with trajectories generated from pathplanner
        PathPlannerTrajectory trajectory = getTrajectory(name);
        if(Objects.isNull(trajectory)){throw new NullPointerException();}
        return followTrajectory(name, trajectory);
    }

    private Command followTrajectory(String name, PathPlannerTrajectory trajectory){
        //For use with trajectories generated from a list of poses
        return new PPSwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose, 
            DriveConstants.kDriveKinematics, 
            xController, 
            yController, 
            rotationController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        ).beforeStarting(() -> trajectoryLog.append("Following trajectory " + name)
        ).andThen(() -> trajectoryLog.append("Trajectory " + name +  " Ended"));
    }

    private Command resetOdometry(String initialTrajectory) throws NullPointerException{
        //Resets odometry to the initial position of the given trajectory
        PathPlannerTrajectory trajectory = getTrajectory(initialTrajectory);
        Pose2d initialPose = Objects.isNull(trajectory) ? new Pose2d(0, 0, new Rotation2d()) : trajectory.getInitialPose();
        return new InstantCommand(() -> swerveSubsystem.resetOdometry(initialPose));
    }

    private Command delay(double seconds){
        return new WaitCommand(seconds).beforeStarting(() -> commandLog.append("Wait " + seconds + " seconds"));
    }
    
    private Command getAutonSequence(){
        autonMode = modeChooser.getSelected();
        //Sequence of actions to be performed during the autonomous period
        try{
        switch(autonMode){
            case FORWARD:
               return 
                    new SequentialCommandGroup(
                        resetOdometry("Forward"),
                        followTrajectory("Forward")
                    );
            case BACKWARD:
                return
                    new SequentialCommandGroup(
                        resetOdometry("Backward"),
                        delay(3),
                        followTrajectory("Backward")
                    );
            case FORWARD180:
                return
                    new SequentialCommandGroup(
                        resetOdometry("Forward180"),
                        followTrajectory("Forward180")
                    );
            default:
                return null;
        }
        }catch(NullPointerException e){
            DriverStation.reportError("Was unable to access a trajectory", e.getStackTrace());
            return getBackupSequence();
        }
    }

    private Command getBackupSequence(){
        //Backup sequence in case a trajectory fails to load
        return new SequentialCommandGroup(
            followTrajectory(
                "Up",
                generateTrajectory(
                    constructPoint(0, 0, 0, 90),
                    constructPoint(0, 1, 0, 0)
                )
            )
        );
    }

    private Command getAutonEnd(){
        //Actions to be performed unconditionally after the autonomous sequence has ended (Stop motors)
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }

    public Command getAutonCommand(){
        return getAutonSequence().beforeStarting(delay(delayEntry.getDouble(0.0))).andThen(getAutonEnd());
    }
}
