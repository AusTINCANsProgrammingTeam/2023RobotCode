package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
    //Operator Interface (OI) class containing all control information

    private static final int kDriverJoystickPort = 0;

    public static final class Driver{
        private static enum Button {
            X (1),
            A (2),
            B (3),
            Y (4),
            LB (5), // Left Bumper
            RB (6), // Right Bumper
            LT (7), // Left Trigger
            RT (8), // Right Trigger
            Back (9),
            Start (10),
            LJ (11), // Left Joystick Button
            RJ (12); // Right Joystick Button
            
            private final int buttonID; 
            private String buttonAction;
          
            Button(int ID) {
              this.buttonID = ID;
              this.buttonAction = "";
            }
          
            private int getButtonID(){
              return this.buttonID;
            };
    
            private String getButtonAction(){
                return this.buttonAction;
            }
    
            private void setButtonAction(String name){
                this.buttonAction = name;
            }
        };

        private static final Joystick kJoystick = new Joystick(OI.kDriverJoystickPort);
        
          private static final Button kOrientationButton = Button.X; //1, Toggle swerve orientation
          private static final Button kZeroButton = Button.B; //3, Zero the gyroscope
          private static final Button kAlignForwardButton = Button.Y; //4, Align forwards
          private static final Button kAlignBackwardButton = Button.A; //2, Align backwards
          private static final Button kIntakeButton = Button.RT; //Right Trigger, run intake
          private static final Button kOuttakeButton = Button.RB; //Right Bumper, run outtake
          private static final Button kParkButton = Button.RB; //5, park the robot
          private static final Button kToggleBalanceButton = Button.LB; //6, balance the robot

        private static final int kXTranslationAxis = 0;
        private static final int kYTranslationAxis = 1;
        private static final int kRotationAxis = 2;

        //TODO: Tune curves to driver preference
        private static final ControlCurve kXTranslationCurve = new ControlCurve(0.65,0.05,0.75,0.1);
        private static final ControlCurve kYTranslationCurve = new ControlCurve(0.65,0.05,0.75,0.1);
        private static final ControlCurve kRotationCurve = new ControlCurve(1,0,1,0.1);

        public static Supplier<Double> getXTranslationSupplier(){
            //This axis is inverted
            return () -> kXTranslationCurve.calculate(-kJoystick.getRawAxis(kXTranslationAxis));
        }

        public static Supplier<Double> getYTranslationSupplier(){
            //This axis is inverted
            return () -> kYTranslationCurve.calculate(-kJoystick.getRawAxis(kYTranslationAxis));
        }

        public static Supplier<Double> getRotationSupplier(){
            //This axis is inverted
            return () -> kRotationCurve.calculate(-kJoystick.getRawAxis(kRotationAxis));
        }

        public static JoystickButton getOrientationButton(){
            kOrientationButton.setButtonAction("Orientation");
            return new JoystickButton(kJoystick, kOrientationButton.getButtonID());
        }

        public static JoystickButton getAlignForwardButton(){
            kAlignForwardButton.setButtonAction("Align Forward");
            return new JoystickButton(kJoystick, kAlignForwardButton.getButtonID());
        }

        public static JoystickButton getAlignBackButton(){
            kAlignBackwardButton.setButtonAction("Align Backward");
            return new JoystickButton(kJoystick, kAlignBackwardButton.getButtonID());
        }

        public static JoystickButton getZeroButton(){
            kZeroButton.setButtonAction("Zeroing");
            return new JoystickButton(kJoystick, kZeroButton.getButtonID());
        }
        
        public static JoystickButton getBalanceButton(){
            kToggleBalanceButton.setButtonAction("Balance Robot");
            return new JoystickButton(kJoystick, kToggleBalanceButton.getButtonID());

        }

        public static JoystickButton getIntakeButton(){
            kIntakeButton.setButtonAction("Intake");
            return new JoystickButton(kJoystick, kIntakeButton.getButtonID());
        }

        public static JoystickButton getOuttakeButton(){
            kOuttakeButton.setButtonAction("Outtake");
            return new JoystickButton(kJoystick, kOuttakeButton.getButtonID());
        }
        
        public static JoystickButton getParkButton(){
            kParkButton.setButtonAction("Parking");
            return new JoystickButton(kJoystick, kParkButton.getButtonID());
        }
    }

    public static final class Operator{
        private static enum Button {
            X (1),
            A (2),
            B (3),
            Y (4),
            LB (5), // Left Bumper
            RB (6), // Right Bumper
            LT (7), // Left Trigger
            RT (8), // Right Trigger
            Back (9),
            Start (10),
            LJ (11), // Left Joystick Button
            RJ (12); // Right Joystick Button
            
            private final int buttonID; 
            private String buttonAction;
          
            Button(int ID) {
              this.buttonID = ID;
              this.buttonAction = "";
            }
          
            private int getButtonID(){
              return this.buttonID;
            };
    
            private String getButtonAction(){
                return this.buttonAction;
            }
    
            private void setButtonAction(String name){
                this.buttonAction = name;
            }
        };
    }

    public static void putControllerButtons(){
        ShuffleboardLayout driverButtonsLayout = Shuffleboard.getTab("Controller Buttons")
        .getLayout("Driver Buttons", BuiltInLayouts.kList)
        .withSize(2, 5)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;

        ShuffleboardLayout operatorButtonsLayout = Shuffleboard.getTab("Controller Buttons")
        .getLayout("Operator Buttons", BuiltInLayouts.kList)
        .withSize(2, 5)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;

        for (Driver.Button button : Driver.Button.values()) {
            driverButtonsLayout.add(String.valueOf(button.getButtonID()), "Button " + button.toString() + ": " + button.getButtonAction());
        }

        for (Operator.Button button : Operator.Button.values()) {
            operatorButtonsLayout.add(String.valueOf(button.getButtonID()+Operator.Button.values().length), "Button " + button.toString() + ": " + button.getButtonAction());
        }
    }
    public static class ControlCurve{
        private double ySaturation; // Maximum output, in percentage of possible output
        private double yIntercept; // Minimum output, in percentage of saturation
        private double curvature; // Curvature shift between linear and cubic
        private double deadzone; // Range of input that will always return zero output

        public ControlCurve(double ySaturation, double yIntercept, double curvature, double deadzone){
            this.ySaturation = ySaturation;
            this.yIntercept = yIntercept;
            this.curvature = curvature;
            this.deadzone = deadzone;
        }

        public double calculate(double input){
            /* https://www.desmos.com/calculator/w6ovblmmqj
            Two equations, separated by a ternary
            The first is the deadzone
            y = 0 {|x| < d}
            The second is the curve
            y = a(sign(x) * b + (1 - b) * (c * x^3 + (1 - c) * x)) {|x| >= d}
            Where
            x = input
            y = output
            a = ySaturation
            b = yIntercept
            c = curvature
            d = deadzone
            and 0 <= a,b,c,d < 1 
            */
            return Math.abs(input) <  deadzone ? 0 : 
            ySaturation * (Math.signum(input) * yIntercept + 
            (1 - yIntercept) * (curvature * Math.pow(input, 3) +
            (1 - curvature) * input));
        }
    }
}
