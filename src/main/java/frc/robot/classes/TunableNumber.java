package frc.robot.classes;

import java.util.function.Consumer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TunableNumber extends SubsystemBase{
    private static ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
    private GenericEntry numberEntry;
    private static GenericEntry istuningMode = tuningTab.add("Tuning Mode", true).getEntry();
    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry numberLog;
    private String name;
    private double value;
    private Consumer<Double> consumer;
    private boolean tuningMode;

    public TunableNumber(String name, double defaultValue, Consumer<Double> consumer){
        this.name = name;
        this.value = defaultValue;
        this.consumer = consumer;
        if (!Robot.isCompetition){
            tuningMode = istuningMode.getBoolean(true);
            if (tuningMode){
                numberEntry = tuningTab.add(name, defaultValue).getEntry();
            }
        }
        numberLog = new DoubleLogEntry(datalog, "/tunables/" + name); 
    }

    public void periodic(){
        if (tuningMode || !Robot.isCompetition){
            double newValue = numberEntry.getDouble(value);
            if (value != newValue){
                value = newValue;
                consumer.accept(value);
                numberLog.append(value);
            }
        }
        
    }

    public double get(){
        return value;
    }

}

