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
    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry numberLog;
    private double value;
    private Consumer<Double> consumer;

    public TunableNumber(String name, double defaultValue, Consumer<Double> consumer){
        this.value = defaultValue;
        this.consumer = consumer;
        if (!Robot.isCompetition){
            numberEntry = tuningTab.add(name, defaultValue).getEntry();
        }
        numberLog = new DoubleLogEntry(datalog, "/tunables/" + name); 
        numberLog.append(defaultValue);
    }

    public void periodic(){
        if (!Robot.isCompetition){
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

