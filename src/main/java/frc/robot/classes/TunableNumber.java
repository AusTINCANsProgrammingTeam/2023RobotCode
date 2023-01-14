package frc.robot.classes;

import java.util.function.Consumer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TunableNumber extends SubsystemBase{
    private static ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
    private GenericEntry numberEntry;
    private static GenericEntry istuningMode = tuningTab.add("Tuning Mode", true).getEntry();
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
    }

    public void periodic(){
        if (!Robot.isCompetition){
            if (tuningMode){
            value = numberEntry.getDouble(0);
            consumer.accept(value);
            }
        }
    }

    public double get(){
        return value;
    }

}

