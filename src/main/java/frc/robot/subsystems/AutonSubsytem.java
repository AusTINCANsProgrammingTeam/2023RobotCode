package frc.robot.subsystems;

import java.util.Objects;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
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
import frc.robot.classes.FieldConstants;


public class AutonSubsytem extends SubsystemBase{
    public static final double kMaxSpeed = SwerveSubsystem.kPhysicalMaxSpeed / 4; //Maximum speed allowed in auton, in meters per second
    public static final double kMaxAcceleration = 3; //Maximum accelaration allowed in auton, in meters per seconds squared
    public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeed, kMaxAcceleration);

    private enum AutonModes{
        FORWARD, // Go forward 1 meter
        BACKWARD, // Wait 3 seconds, go backward 1 meter
        FORWARD180, // Go forward 2 meters and rotate 180 degrees
        CURVE; // Go forward 1 meter and left 1 meter
    }
    private final AutonModes kDefaultAutonMode = AutonModes.FORWARD;

    private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
    private GenericEntry delayEntry = configTab.add("Auton Delay", 0.0).getEntry();
    private SendableChooser<AutonModes> modeChooser = new SendableChooser<>();

    private DataLog datalog = DataLogManager.getLog();
    private StringLogEntry commandLog = new StringLogEntry(datalog, "/auton/command"); //Logs x translation state output

    private SwerveSubsystem swerveSubsystem;

    private AutonModes autonMode;

    public AutonSubsytem(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        //Add auton modes to chooser
        for(AutonModes mode : AutonModes.values()){
            modeChooser.addOption(mode.toString(), mode);
        }
        modeChooser.setDefaultOption(kDefaultAutonMode.toString(), kDefaultAutonMode);
        configTab.add("Auton mode", modeChooser);
    }

    public static PathPlannerTrajectory getTrajectory(String name) throws NullPointerException{
        return PathPlanner.loadPath(name, kPathConstraints);
    }

    public static PathPoint constructPoint(double x, double y, double rotation, double heading){
        return new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(heading), Rotation2d.fromDegrees(rotation));
    }

    public static PathPoint constructPoint(Pose2d pose, double heading){
        return new PathPoint(pose.getTranslation(), Rotation2d.fromDegrees(heading), pose.getRotation());
    }

    public static PathPlannerTrajectory generateTrajectory(PathPoint firstPoint, PathPoint secondPoint, PathPoint... points){
        return PathPlanner.generatePath(
            kPathConstraints,
            firstPoint,
            secondPoint,
            points
        );
    }

    private Command resetOdometry(String initialTrajectory) throws NullPointerException{
        //Resets odometry to the initial position of the given trajectory
        PathPlannerTrajectory trajectory = getTrajectory(initialTrajectory);
        Pose2d initialPose = FieldConstants.odometryFlip(Objects.isNull(trajectory) ? new Pose2d(0, 0, new Rotation2d()) : trajectory.getInitialPose());
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
                        swerveSubsystem.followTrajectory("Forward", getTrajectory("lol"))
                    );
            case BACKWARD:
                return
                    new SequentialCommandGroup(
                        resetOdometry("Backward"),
                        delay(3),
                        swerveSubsystem.followTrajectory("Backward", getTrajectory("Backward"))
                    );
            case FORWARD180:
                return
                    new SequentialCommandGroup(
                        resetOdometry("Forward180"),
                        swerveSubsystem.followTrajectory("Forward180", getTrajectory("Forward180"))
                    );
            case CURVE:
                return
                    new SequentialCommandGroup(
                        resetOdometry("Curve"),
                        swerveSubsystem.followTrajectory("Curve", getTrajectory("Curve"))
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
            swerveSubsystem.followTrajectory(
                "Apriltag",
                generateTrajectory(
                    constructPoint(0, 0, 0, 0),
                    constructPoint(
                    FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(3)).getTranslation().getX(),
                    FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(3)).getTranslation().getY(),0,0)

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
