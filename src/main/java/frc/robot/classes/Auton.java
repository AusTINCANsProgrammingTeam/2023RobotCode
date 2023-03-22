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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CubeapultSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Auton{
    public static final double kMaxSpeed = SwerveSubsystem.kPhysicalMaxSpeed * 0.75; //Maximum speed allowed in auton, in meters per second
    public static final double kMaxAcceleration = 3; //Maximum accelaration allowed in auton, in meters per seconds squared

    private enum AutonModes{
        //Number after a path name corresponds to its starting position
        //Test routines
        PLACETEST, LAUNCHTEST,
        //Skip scoring and balance
        CHARGE1, CHARGE6,
        //Score preload and drive out of the community
        ONESCORE,
        //Score preload and balance
        ONESCORECHARGE1, ONESCORECHARGE2, ONESCORECHARGE3, ONESCORECHARGE4, ONESCORECHARGE5, ONESCORECHARGE6,
        //Score preload then score another game piece
        TWOSCORE1, TWOSCORE6,
        //Score preload, score another game piece, then balance
        TWOSCORECHARGE1, TWOSCORECHARGE6,
        //Score preload, score another game piece, intake another game piece, then balance
        TWOSCORELOADCHARGE1, TWOSCORELOADCHARGE6,
        //Score preload, then score two other game pieces
        THREESCORE1, THREESCORE6,
        //Score preload, score two other game pieces, then balance
        THREESCORECHARGE1, THREESCORECHARGE6
    }
    private final AutonModes kDefaultAutonMode = AutonModes.ONESCORE;

    private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
    private GenericEntry delayEntry = configTab.add("Auton Delay", 0.0).getEntry();
    private SendableChooser<AutonModes> modeChooser = new SendableChooser<>();

    private DataLog datalog = DataLogManager.getLog();
    private StringLogEntry commandLog = new StringLogEntry(datalog, "/auton/command"); //Logs x translation state output

    private SwerveSubsystem swerveSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private CubeapultSubsystem cubeapultSubsystem;

    private PathConstraints pathConstraints;

    private AutonModes autonMode;

    public Auton(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, CubeapultSubsystem cubeapultSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.cubeapultSubsystem = cubeapultSubsystem;

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

    private PathPlannerTrajectory getTrajectory(String name, PathConstraints pathConstraints) throws NullPointerException{
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
        Pose2d initialPose = FieldConstants.allianceFlip(Objects.isNull(trajectory) ? new Pose2d(0, 0, new Rotation2d()) : trajectory.getInitialHolonomicPose());
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.zeroHeading(initialPose.getRotation())),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(initialPose))
        );
    }

    private Command highScoreSequence() {
        return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
            armSubsystem.goToState(ArmState.HIGHTRANSITION),
            armSubsystem.goToState(ArmState.HIGHSCORE)
            ), 
            new StartEndCommand(intakeSubsystem::pull, intakeSubsystem::stop, intakeSubsystem)
        ),
        armSubsystem.goToState(ArmState.HIGHDROP)
        );
    }

    private Command delay(double seconds){
        return new WaitCommand(seconds).beforeStarting(() -> commandLog.append("Wait " + seconds + " seconds"));
    }
    
    private Command getAutonSequence(){
        autonMode = modeChooser.getSelected();
        //Sequence of actions to be performed during the autonomous period
        try{
        switch(autonMode){
            case PLACETEST:
                return 
                    new SequentialCommandGroup(
                        resetOdometry("1Score"),
                        highScoreSequence(),
                        new InstantCommand(() -> SmartDashboard.putBoolean("next", true))
                    );
            case LAUNCHTEST:
                return
                    new SequentialCommandGroup(
                        cubeapultSubsystem.launch()
                    );
            case ONESCORE:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1Score"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("1Score", getTrajectory("1Score"))
                        .deadlineWith(armSubsystem.stowArmParallel())
                    );
            case CHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-1"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-1", getTrajectory("1ScoreCharge-1")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case CHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-6"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-6", getTrajectory("1ScoreCharge-6")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case ONESCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-1"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-1", getTrajectory("1ScoreCharge-1"))
                        .deadlineWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.assistedBalance(true)
                    );
            case ONESCORECHARGE2:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-2"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-2", getTrajectory("1ScoreCharge-2"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.assistedBalance(true)
                    );
            case ONESCORECHARGE3:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-3"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-3", getTrajectory("1ScoreCharge-3"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.assistedBalance(true)
                    );
            case ONESCORECHARGE4:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-4"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-4", getTrajectory("1ScoreCharge-4"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.assistedBalance(true)
                    );
            case ONESCORECHARGE5:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-5"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-5", getTrajectory("1ScoreCharge-5"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.assistedBalance(true)
                    );
            case ONESCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-6"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-6", getTrajectory("1ScoreCharge-6"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.assistedBalance(true)
                    );
            case TWOSCORE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2Score1-1"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("2Score1-1", getTrajectory("2Score1-1"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("2Score2-1", getTrajectory("2Score2-1"))
                    );
            case TWOSCORE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2Score1-6"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("2Score1-6", getTrajectory("2Score1-6"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("2Score2-6", getTrajectory("2Score2-6"))
                    );
            case TWOSCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreCharge1-1"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("2ScoreCharge1-1", getTrajectory("2ScoreCharge1-1"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("2ScoreCharge2-1", getTrajectory("2ScoreCharge2-1")),
                        swerveSubsystem.followTrajectory("2ScoreCharge3-1", getTrajectory("2ScoreCharge3-1")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case TWOSCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreCharge1-6"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("2ScoreCharge1-6", getTrajectory("2ScoreCharge1-6"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("2ScoreCharge2-6", getTrajectory("2ScoreCharge2-6")),
                        swerveSubsystem.followTrajectory("2ScoreCharge3-6", getTrajectory("2ScoreCharge3-6")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case THREESCORE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3Score1-1"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("3Score1-1", getTrajectory("3Score1-1"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("3Score2-1", getTrajectory("3Score2-1")),
                        swerveSubsystem.followTrajectory("3Score3-1", getTrajectory("3Score3-1")),
                        swerveSubsystem.followTrajectory("3Score4-1", getTrajectory("3Score4-1"))
                    );
            case THREESCORE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3Score1-6"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("3Score1-6", getTrajectory("3Score1-6"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("3Score2-6", getTrajectory("3Score2-6")),
                        swerveSubsystem.followTrajectory("3Score3-6", getTrajectory("3Score3-6")),
                        swerveSubsystem.followTrajectory("3Score4-6", getTrajectory("3Score4-6"))
                    );
            case TWOSCORELOADCHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreLoadCharge1-1"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge1-1", getTrajectory("2ScoreLoadCharge1-1"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge2-1", getTrajectory("2ScoreLoadCharge2-1")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge3-1", getTrajectory("2ScoreLoadCharge3-1")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge4-1", getTrajectory("2ScoreLoadCharge4-1")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case TWOSCORELOADCHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreLoadCharge1-6"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge1-6", getTrajectory("2ScoreLoadCharge1-6"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge2-6", getTrajectory("2ScoreLoadCharge2-6")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge3-6", getTrajectory("2ScoreLoadCharge3-6")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge4-6", getTrajectory("2ScoreLoadCharge4-6")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case THREESCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3ScoreCharge1-1"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("3ScoreCharge1-1", getTrajectory("3ScoreCharge1-1"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("3ScoreCharge2-1", getTrajectory("3ScoreCharge2-1")),
                        swerveSubsystem.followTrajectory("3ScoreCharge3-1", getTrajectory("3ScoreCharge3-1")),
                        swerveSubsystem.followTrajectory("3ScoreCharge4-1", getTrajectory("3ScoreCharge4-1")),
                        swerveSubsystem.followTrajectory("3ScoreCharge5-1", getTrajectory("3ScoreCharge5-1")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case THREESCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3ScoreCharge1-6"),
                        highScoreSequence(),
                        swerveSubsystem.followTrajectory("3ScoreCharge1-6", getTrajectory("3ScoreCharge1-6"))
                        .alongWith(armSubsystem.stowArmParallel()),
                        swerveSubsystem.followTrajectory("3ScoreCharge2-6", getTrajectory("3ScoreCharge2-6")),
                        swerveSubsystem.followTrajectory("3ScoreCharge3-6", getTrajectory("3ScoreCharge3-6")),
                        swerveSubsystem.followTrajectory("3ScoreCharge4-6", getTrajectory("3ScoreCharge4-6")),
                        swerveSubsystem.followTrajectory("3ScoreCharge5-6", getTrajectory("3ScoreCharge5-6")),
                        swerveSubsystem.assistedBalance(true)
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
            highScoreSequence()
        );
    }

    private Command getAutonEnd(){
        //Actions to be performed unconditionally after the autonomous sequence has ended (Stop motors)
        return new SequentialCommandGroup(
            new InstantCommand(swerveSubsystem::stopModules)
        );
    }

    public Command getAutonCommand(){
        return getAutonSequence().beforeStarting(delay(delayEntry.getDouble(0.0))).andThen(getAutonEnd());
    }
}