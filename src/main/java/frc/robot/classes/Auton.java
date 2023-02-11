package frc.robot.classes;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class Auton{
    public static final double kMaxSpeed = SwerveSubsystem.kPhysicalMaxSpeed / 4; //Maximum speed allowed in auton, in meters per second
    public static final double kMaxAcceleration = 3; //Maximum accelaration allowed in auton, in meters per seconds squared

    private enum AutonModes{
        FORWARD, // Go forward 1 meter
        BACKWARD, // Wait 3 seconds, go backward 1 meter
        FORWARD180, // Go forward 2 meters and rotate 180 degrees
        CURVE, // Go forward 1 meter and left 1 meter
        ONESCORECHARGE1, // Score preloaded game piece from first starting position and engage charge pad
        ONESCORECHARGE2, // Score preloaded game piece from second starting position and engage charge pad
        ONESCORECHARGE3, // Score preloaded game piece from third starting position and engage charge pad
        ONESCORECHARGE4, // Score preloaded game piece from fourth starting position and engage charge pad
        ONESCORECHARGE5, // Score preloaded game piece from fifth starting position and engage charge pad
        ONESCORECHARGE6, // Score preloaded game piece from sixth starting position and engage charge pad
        TWOSCORE1, // Score preloaded game piece from first starting position and score another game piece
        TWOSCORE6, // Score preloaded game piece from sixth starting position and score another game piece
        TWOSCORECHARGE1, // Score preloaded game piece from first starting position, score another game piece, and engage charge pad
        TWOSCORECHARGE6, // Score preloaded game piece from sixth starting position, score another game piece, and engage charge pad
        THREESCORE1, // Score preloaded game piece from first starting position, score another game piece, and score another game piece
        THREESCORE6, // Score preloaded game piece from sixth starting position, score another game piece, and score another game piece
        TWOSCORELOADCHARGE1, // Score preloaded game piece from first starting position, score another game piece, intake another game piece, and engage charge pad
        TWOSCORELOADCHARGE6, // Score preloaded game piece from sixth starting position, score another game piece, intake another game piece, and engage charge pad
        THREESCORECHARGE1, // Score preloaded game piece from first starting position, score another game piece, score another game piece, and engage charge pad
        THREESCORECHARGE6, // Score preloaded game piece from sixth starting position, score another game piece, score another game piece, and engage charge pad
    }
    private final AutonModes kDefaultAutonMode = AutonModes.FORWARD;

    private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
    private GenericEntry delayEntry = configTab.add("Auton Delay", 0.0).getEntry();
    private SendableChooser<AutonModes> modeChooser = new SendableChooser<>();

    private DataLog datalog = DataLogManager.getLog();
    private StringLogEntry commandLog = new StringLogEntry(datalog, "/auton/command"); //Logs x translation state output

    private SwerveSubsystem swerveSubsystem;

    private PathConstraints pathConstraints;

    private AutonModes autonMode;

    public Auton(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        //Add auton modes to chooser
        for(AutonModes mode : AutonModes.values()){
            modeChooser.addOption(mode.toString(), mode);
        }
        modeChooser.setDefaultOption(kDefaultAutonMode.toString(), kDefaultAutonMode);
        configTab.add("Auton mode", modeChooser);

        pathConstraints = new PathConstraints(kMaxSpeed, kMaxAcceleration);
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

    private Command resetOdometry(String initialTrajectory) throws NullPointerException{
        //Resets odometry to the initial position of the given trajectory
        PathPlannerTrajectory trajectory = getTrajectory(initialTrajectory);
        Pose2d initialPose = FieldConstants.allianceFlip(Objects.isNull(trajectory) ? new Pose2d(0, 0, new Rotation2d()) : trajectory.getInitialPose());
        return new InstantCommand(() -> swerveSubsystem.resetOdometry(initialPose)).alongWith(new InstantCommand(() -> swerveSubsystem.zeroHeading(initialPose.getRotation())));
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
                        swerveSubsystem.followTrajectory("Forward", getTrajectory("Forward"))
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
            case ONESCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-1"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-1", getTrajectory("1ScoreCharge-1"))
                    );
            case ONESCORECHARGE2:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-2"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-2", getTrajectory("1ScoreCharge-2"))
                    );
            case ONESCORECHARGE3:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-3"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-3", getTrajectory("1ScoreCharge-3"))
                    );
            case ONESCORECHARGE4:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-4"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-4", getTrajectory("1ScoreCharge-4"))
                    );
            case ONESCORECHARGE5:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-5"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-5", getTrajectory("1ScoreCharge-5"))
                    );
            case ONESCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-6"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-6", getTrajectory("1ScoreCharge-6"))
                    );
            case TWOSCORE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2Score1-1"),
                        swerveSubsystem.followTrajectory("2Score1-1", getTrajectory("2Score1-1")),
                        swerveSubsystem.followTrajectory("2Score2-1", getTrajectory("2Score2-1"))
                    );
            case TWOSCORE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2Score1-6"),
                        swerveSubsystem.followTrajectory("2Score1-6", getTrajectory("2Score1-6")),
                        swerveSubsystem.followTrajectory("2Score2-6", getTrajectory("2Score2-6"))
                    );
            case TWOSCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreCharge1-1"),
                        swerveSubsystem.followTrajectory("2ScoreCharge1-1", getTrajectory("2ScoreCharge1-1")),
                        swerveSubsystem.followTrajectory("2ScoreCharge2-1", getTrajectory("2ScoreCharge2-1")),
                        swerveSubsystem.followTrajectory("2ScoreCharge3-1", getTrajectory("2ScoreCharge3-1"))
                    );
            case TWOSCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreCharge1-6"),
                        swerveSubsystem.followTrajectory("2ScoreCharge1-6", getTrajectory("2ScoreCharge1-6")),
                        swerveSubsystem.followTrajectory("2ScoreCharge2-6", getTrajectory("2ScoreCharge2-6")),
                        swerveSubsystem.followTrajectory("2ScoreCharge3-6", getTrajectory("2ScoreCharge3-6"))
                    );
            case THREESCORE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3Score1-1"),
                        swerveSubsystem.followTrajectory("3Score1-1", getTrajectory("3Score1-1")),
                        swerveSubsystem.followTrajectory("3Score2-1", getTrajectory("3Score2-1")),
                        swerveSubsystem.followTrajectory("3Score3-1", getTrajectory("3Score3-1")),
                        swerveSubsystem.followTrajectory("3Score4-1", getTrajectory("3Score4-1"))
                    );
            case THREESCORE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3Score1-6"),
                        swerveSubsystem.followTrajectory("3Score1-6", getTrajectory("3Score1-6")),
                        swerveSubsystem.followTrajectory("3Score2-6", getTrajectory("3Score2-6")),
                        swerveSubsystem.followTrajectory("3Score3-6", getTrajectory("3Score3-6")),
                        swerveSubsystem.followTrajectory("3Score4-6", getTrajectory("3Score4-6"))
                    );
            case TWOSCORELOADCHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreLoadCharge1-1"),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge1-1", getTrajectory("2ScoreLoadCharge1-1")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge2-1", getTrajectory("2ScoreLoadCharge2-1")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge3-1", getTrajectory("2ScoreLoadCharge3-1")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge4-1", getTrajectory("2ScoreLoadCharge4-1"))
                    );
            case TWOSCORELOADCHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreLoadCharge1-6"),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge1-6", getTrajectory("2ScoreLoadCharge1-6")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge2-6", getTrajectory("2ScoreLoadCharge2-6")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge3-6", getTrajectory("2ScoreLoadCharge3-6")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge4-6", getTrajectory("2ScoreLoadCharge4-6"))
                    );
            case THREESCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3ScoreCharge1-1"),
                        swerveSubsystem.followTrajectory("3ScoreCharge1-1", getTrajectory("3ScoreCharge1-1")),
                        swerveSubsystem.followTrajectory("3ScoreCharge2-1", getTrajectory("3ScoreCharge2-1")),
                        swerveSubsystem.followTrajectory("3ScoreCharge3-1", getTrajectory("3ScoreCharge3-1")),
                        swerveSubsystem.followTrajectory("3ScoreCharge4-1", getTrajectory("3ScoreCharge4-1")),
                        swerveSubsystem.followTrajectory("3ScoreCharge5-1", getTrajectory("3ScoreCharge5-1"))
                    );
            case THREESCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3ScoreCharge1-6"),
                        swerveSubsystem.followTrajectory("3ScoreCharge1-6", getTrajectory("3ScoreCharge1-6")),
                        swerveSubsystem.followTrajectory("3ScoreCharge2-6", getTrajectory("3ScoreCharge2-6")),
                        swerveSubsystem.followTrajectory("3ScoreCharge3-6", getTrajectory("3ScoreCharge3-6")),
                        swerveSubsystem.followTrajectory("3ScoreCharge4-6", getTrajectory("3ScoreCharge4-6")),
                        swerveSubsystem.followTrajectory("3ScoreCharge5-6", getTrajectory("3ScoreCharge5-6"))
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