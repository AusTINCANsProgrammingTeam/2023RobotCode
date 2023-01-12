package frc.robot.classes;

import java.util.function.Consumer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TunableNumber extends SubsystemBase{
    private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
    private GenericEntry numberEntry;
    private String name;
    private double value;
    private Consumer<Double> consumer;

    public TunableNumber(String name, double defaultValue, Consumer<Double> consumer){
        this.name = name;
        this.value = defaultValue;
        this.consumer = consumer;
        if (Robot.tuningMode){
            numberEntry = tuningTab.add(name, defaultValue).getEntry();
        }
    }

    public void periodic(){
        if (Robot.tuningMode){
            value = numberEntry.getDouble(0);
            consumer.accept(value);
        }
    }

    public double get(){
        return value;
    }

}

