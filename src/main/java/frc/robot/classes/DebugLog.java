package frc.robot.classes;

import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DebugLog<T> {
    private static DataLog datalog = DataLogManager.getLog();

    private ShuffleboardTab networkTab;
    private GenericEntry networkEntry;
    private Integer localEntry;
    private Consumer<T> localConsumer;
    private String name;

    public DebugLog(T defaultValue, String name, SubsystemBase subsystem){
        this.name = name;
        if(defaultValue instanceof Double){
            localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "double");
            localConsumer = (a) -> datalog.appendDouble(localEntry, (Double)a, 0);
        }
        if(defaultValue instanceof String){
            localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "string");
            localConsumer = (a) -> datalog.appendString(localEntry, (String)a, 0);
        }
        if(defaultValue instanceof Boolean){
            localEntry = datalog.start("/" + subsystem.getName() + "/" + name, "boolean");
            localConsumer = (a) -> datalog.appendBoolean(localEntry, (Boolean)a, 0);
        }
        if (localEntry != null) {
            if(!Robot.isCompetition){
                networkTab = Shuffleboard.getTab(subsystem.getName());
                networkEntry = networkTab.add(name, defaultValue).getEntry();
            }
            localConsumer.accept(defaultValue);
        }
    }

    public void log(T newValue){
        try{
            if(!Robot.isCompetition){
                networkEntry.setValue(newValue);
            }
            localConsumer.accept(newValue);
        } catch(NullPointerException e){
            DriverStation.reportError("Invalid type for log " + name, false);
        }
    }
}