package frc.robot.classes;

import java.util.HashMap;
import java.util.Objects;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
        PLACETEST, LAUNCHTEST, TESTOVERIDE,
        //Skip scoring and balance
        CHARGE1, CHARGE6,
        //Score preload and drive out of the community
        ONESCORE,
        //Score preload and balance
        ONESCORECHARGE1, ONESCORECHARGE2, ONESCORECHARGE3, ONESCORECHARGE4, ONESCORECHARGE5, ONESCORECHARGE6,
        //Score preload, intake another game piece, then balance
        ONESCORELOADCHARGE1, ONESCORELOADCHARGE6, ONECUBELOAD1, ONECUBELOAD6,
        //Score preload then score another game piece
        TWOSCORE1, TWOSCORECUBE1, TWOSCORE6,
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
    private HashMap<String, Command> actions;

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
        configTab.add("Auton mode", modeChooser).withSize(2, 1);

        pathConstraints = new PathConstraints(kMaxSpeed, kMaxAcceleration);

        actions = new HashMap<>();
        actions.put("armConeIntake", armSubsystem.transitionToState(ArmState.CONEINTAKE));
        actions.put("armCubeIntake", armSubsystem.transitionToState(ArmState.CUBEINTAKE));
        actions.put("cubePullTransition", intakeSubsystem.pullTimed(1.5, false).andThen(highTransitionSequenceCube()));
        actions.put("cubePullIntake", intakeSubsystem.pullTimed(1.5, false).andThen(armSubsystem.transitionToState(ArmState.CUBEINTAKE)));
        actions.put("cubeScore", scoreSequenceCube());
        actions.put("conePull", intakeSubsystem.pullTimed(1.5, true).andThen(armSubsystem.goToState(ArmState.STOWED)));
        actions.put("conePullTransition", intakeSubsystem.pullTimed(1.5, true).andThen(highTransitionSequenceCone()));
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

    private Command highScoreSequenceCone() {
        return new SequentialCommandGroup(
        new InstantCommand(intakeSubsystem::setConeMode),
        new ParallelDeadlineGroup(
            armSubsystem.goToState(ArmState.HIGHSCORECONE),
            new StartEndCommand(intakeSubsystem::pull, intakeSubsystem::stop, intakeSubsystem)
        ),
        new WaitCommand(1).deadlineWith(new RepeatCommand(new InstantCommand(armSubsystem::updateMotors))),
        armSubsystem.highDrop()
        );
    }

    private Command highTransitionSequenceCube() {
        return new SequentialCommandGroup(
        new InstantCommand(intakeSubsystem::setCubeMode),
        new ParallelDeadlineGroup(
            armSubsystem.goToState(ArmState.MIDSCORECUBE),
            new StartEndCommand(intakeSubsystem::pull, intakeSubsystem::stop, intakeSubsystem)
        )
        );
    }

    private Command highTransitionSequenceCone() {
        return new SequentialCommandGroup(
        new InstantCommand(intakeSubsystem::setConeMode),
        new ParallelDeadlineGroup(
            armSubsystem.goToState(ArmState.HIGHTRANSITIONAUTON),
            new StartEndCommand(intakeSubsystem::pull, intakeSubsystem::stop, intakeSubsystem)
        )
        );
    }

    private Command scoreSequenceCube() {
        return new SequentialCommandGroup(
        new InstantCommand(intakeSubsystem::setCubeMode),
        intakeSubsystem.pushTimed(1, false)
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
                        highScoreSequenceCone(),
                        new InstantCommand(() -> SmartDashboard.putBoolean("next", true))
                    );
            case LAUNCHTEST:
                return
                    new SequentialCommandGroup(
                        cubeapultSubsystem.launch()
                    );
            case TESTOVERIDE:
                return
                    new SequentialCommandGroup(
                        resetOdometry("TestVelocityOveride"),
                        swerveSubsystem.followTrajectory("TestVelocityOveride", getTrajectory("TestVelocityOveride"))
                    );
            case ONESCORE:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1Score"),
                        highTransitionSequenceCone(),
                        new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                        resetOdometry("1Score"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("1Score", getTrajectory("1Score"))
                        .deadlineWith(armSubsystem.goToStateDelay(ArmState.STOWED).andThen(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors()))))
                    );
            case CHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-1"),
                        swerveSubsystem.followTrajectory("1ScoreCharge-1", getTrajectory("1ScoreCharge-1"))
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
                        highTransitionSequenceCone(),
                        new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                        resetOdometry("1ScoreCharge-1"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-1", getTrajectory("1ScoreCharge-1"))
                        .deadlineWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.assistedBalance(true).deadlineWith(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))
                    );
            case ONESCORECHARGE2:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-2"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-2", getTrajectory("1ScoreCharge-2"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.assistedBalance(true)
                    );
            case ONESCORECHARGE3:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-3"),
                        highTransitionSequenceCone(),
                        new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                        resetOdometry("1ScoreCharge-3"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-3", getTrajectory("1ScoreCharge-3"))
                        .deadlineWith(armSubsystem.goToStateDelay(ArmState.STOWED).andThen(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))),
                        swerveSubsystem.assistedBalance(true).deadlineWith(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))
                    );
            case ONESCORECHARGE4:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-4"),
                        highTransitionSequenceCone(),
                        new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                        resetOdometry("1ScoreCharge-4"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-4", getTrajectory("1ScoreCharge-4"))
                        .deadlineWith(armSubsystem.goToStateDelay(ArmState.STOWED).andThen(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))),
                        swerveSubsystem.assistedBalance(true).deadlineWith(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))
                    );
            case ONESCORECHARGE5:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-5"),
                        highTransitionSequenceCone(),
                        new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                        resetOdometry("1ScoreCharge-5"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-5", getTrajectory("1ScoreCharge-5"))
                        .deadlineWith(armSubsystem.goToStateDelay(ArmState.STOWED).andThen(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))),
                        swerveSubsystem.assistedBalance(true).deadlineWith(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))
                    );
            case ONESCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("1ScoreCharge-6"),
                        highTransitionSequenceCone(),
                        new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                        resetOdometry("1ScoreCharge-6"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("1ScoreCharge-6", getTrajectory("1ScoreCharge-6"))
                        .deadlineWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.assistedBalance(true).deadlineWith(new RepeatCommand(new InstantCommand(() -> armSubsystem.updateMotors())))
                    );
            case ONESCORELOADCHARGE1:
            return
                new SequentialCommandGroup(
                    resetOdometry("1ScoreConeLoad1-1"),
                    highTransitionSequenceCone(),
                    new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                    resetOdometry("1ScoreConeLoad1-1"),
                    highScoreSequenceCone(),
                    swerveSubsystem.followTrajectory("1ScoreConeLoad1-1", getTrajectory("1ScoreConeLoad1-1")).deadlineWith(armSubsystem.transitionToState(ArmState.CONEINTAKE)),
                    new ParallelDeadlineGroup(
                        swerveSubsystem.followTrajectory("1ScoreLoad2-1", getTrajectory("1ScoreLoad2-1")),
                        intakeSubsystem.pullTimed(3, true)
                    ).andThen(armSubsystem.goToStateDelay(ArmState.STOWED)),
                    swerveSubsystem.assistedBalance(false)
                );
            case ONESCORELOADCHARGE6:
            return
                new SequentialCommandGroup(
                    resetOdometry("1ScoreConeLoad1-6"),
                    highTransitionSequenceCone(),
                    new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.5),
                    resetOdometry("1ScoreConeLoad1-6"),
                    highScoreSequenceCone(),
                    swerveSubsystem.followTrajectory("1ScoreConeLoad1-6", getTrajectory("1ScoreConeLoad1-6")).deadlineWith(armSubsystem.goToStateDelay(ArmState.CONEINTAKE)),
                    new ParallelDeadlineGroup(
                        swerveSubsystem.followTrajectory("1ScoreLoad2-6", getTrajectory("1ScoreLoad2-6")),
                        intakeSubsystem.pullTimed(3, true)
                    ).andThen(armSubsystem.goToStateDelay(ArmState.STOWED)),
                    swerveSubsystem.assistedBalance(false)
                );        
            case ONECUBELOAD1:
            return
                new SequentialCommandGroup(
                    resetOdometry("1ScoreCubeLoad1-1"),
                    cubeapultSubsystem.launch(),
                    new FollowPathWithEvents(
                        swerveSubsystem.followTrajectory("1ScoreCubeLoad1-1", getTrajectory("1ScoreCubeLoad1-1")), 
                        getTrajectory("1ScoreCubeLoad1-1").getMarkers(),
                        actions
                    ),
                    swerveSubsystem.assistedBalance(false)
                    );
            case ONECUBELOAD6:
            return
                new SequentialCommandGroup(
                    resetOdometry("1ScoreCubeLoad1-6"),
                    cubeapultSubsystem.launch(),
                    swerveSubsystem.followTrajectory("1ScoreCubeLoad1-6", getTrajectory("1ScoreCubeLoad1-6")),
                    new ParallelDeadlineGroup(
                        swerveSubsystem.followTrajectory("1ScoreLoad2-6", getTrajectory("1ScoreLoad2-6")),
                        intakeSubsystem.pullTimed(3, true)
                    ).andThen(armSubsystem.goToStateDelay(ArmState.STOWED)),
                    swerveSubsystem.assistedBalance(false)
                    );            
            case TWOSCORE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2Score-1"),
                        cubeapultSubsystem.launch(),
                        new FollowPathWithEvents(
                            swerveSubsystem.followTrajectory("2Score-1", getTrajectory("2Score-1")), 
                            getTrajectory("2Score-1").getMarkers(),
                            actions
                        ),
                        new StartEndCommand(() -> swerveSubsystem.setModuleStates(swerveSubsystem.convertToModuleStates(0, -0.1, 0)), () -> swerveSubsystem.stopModules()).withTimeout(0.3),
                        highScoreSequenceCone()
                    );
            case TWOSCORECUBE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreCube-1"),
                        cubeapultSubsystem.launch(),
                        new FollowPathWithEvents(
                            swerveSubsystem.followTrajectory("2ScoreCube-1", getTrajectory("2ScoreCube-1")), 
                            getTrajectory("2ScoreCube-1").getMarkers(),
                            actions
                        ),
                        scoreSequenceCube()
                    );
            case TWOSCORE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2Score1-6"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("2Score1-6", getTrajectory("2Score1-6"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.followTrajectory("2Score2-6", getTrajectory("2Score2-6"))
                    );
            case TWOSCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreCharge1-1"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("2ScoreCharge1-1", getTrajectory("2ScoreCharge1-1"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.followTrajectory("2ScoreCharge2-1", getTrajectory("2ScoreCharge2-1")),
                        swerveSubsystem.followTrajectory("2ScoreCharge3-1", getTrajectory("2ScoreCharge3-1")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case TWOSCORECHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreCharge1-6"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("2ScoreCharge1-6", getTrajectory("2ScoreCharge1-6"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.followTrajectory("2ScoreCharge2-6", getTrajectory("2ScoreCharge2-6")),
                        swerveSubsystem.followTrajectory("2ScoreCharge3-6", getTrajectory("2ScoreCharge3-6")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case THREESCORE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3ScoreCube1-1"),
                        cubeapultSubsystem.launch(),
                        new FollowPathWithEvents(
                            swerveSubsystem.followTrajectory("3ScoreCube1-1", getTrajectory("3ScoreCube1-1")), 
                            getTrajectory("3ScoreCube1-1").getMarkers(),
                            actions
                        ),
                        scoreSequenceCube(),
                        new FollowPathWithEvents(
                            swerveSubsystem.followTrajectory("3ScoreCube2-1", getTrajectory("3ScoreCube2-1")), 
                            getTrajectory("3ScoreCube2-1").getMarkers(),
                            actions
                        ),
                        scoreSequenceCube()
                    );
            case THREESCORE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3Score1-6"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("3Score1-6", getTrajectory("3Score1-6"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.followTrajectory("3Score2-6", getTrajectory("3Score2-6")),
                        swerveSubsystem.followTrajectory("3Score3-6", getTrajectory("3Score3-6")),
                        swerveSubsystem.followTrajectory("3Score4-6", getTrajectory("3Score4-6"))
                    );
            case TWOSCORELOADCHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreLoadCharge1-1"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge1-1", getTrajectory("2ScoreLoadCharge1-1"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge2-1", getTrajectory("2ScoreLoadCharge2-1")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge3-1", getTrajectory("2ScoreLoadCharge3-1")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge4-1", getTrajectory("2ScoreLoadCharge4-1")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case TWOSCORELOADCHARGE6:
                return
                    new SequentialCommandGroup(
                        resetOdometry("2ScoreLoadCharge1-6"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge1-6", getTrajectory("2ScoreLoadCharge1-6"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge2-6", getTrajectory("2ScoreLoadCharge2-6")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge3-6", getTrajectory("2ScoreLoadCharge3-6")),
                        swerveSubsystem.followTrajectory("2ScoreLoadCharge4-6", getTrajectory("2ScoreLoadCharge4-6")),
                        swerveSubsystem.assistedBalance(true)
                    );
            case THREESCORECHARGE1:
                return
                    new SequentialCommandGroup(
                        resetOdometry("3ScoreCharge1-1"),
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("3ScoreCharge1-1", getTrajectory("3ScoreCharge1-1"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
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
                        highScoreSequenceCone(),
                        swerveSubsystem.followTrajectory("3ScoreCharge1-6", getTrajectory("3ScoreCharge1-6"))
                        .alongWith(armSubsystem.goToStateDelay(ArmState.STOWED)),
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
            highScoreSequenceCone()
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