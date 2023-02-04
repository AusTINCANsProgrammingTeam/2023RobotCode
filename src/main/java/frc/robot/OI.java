package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI {
    //Operator Interface (OI) class containing all control information

    private static final int kDriverJoystickPort = 0;

    public static final class Driver{
        private static final Joystick kJoystick = new Joystick(OI.kDriverJoystickPort);

        public enum DriverButtons {
            X (1, null),
            A (2, null),
            B (3, null),
            Y (4, null),
            LB (5, null),
            RB (6, null),
            LT (7, null),
            RT (8, null),
            Back (9, null),
            Start (10, null),
            LJ (11, null),
            RJ (12, null);
            
            private final int buttonID; 
            private String buttonName;
          
            DriverButtons(int ID, String N) {
              this.buttonID = ID;
              this.buttonName = N;
            }
          
            private int getButtonID(){
              return this.buttonID;
            };

            private String getButtonName(){
                return this.buttonName;
            }

            private void setButtonName(String name){
                this.buttonName = name;
            }
          };
      

        private static final DriverButtons kOrientationButton = DriverButtons.X; //1 Button, Toggle swerve orientation
        private static final DriverButtons kZeroButton = DriverButtons.A; //2 Button, Zero the gyroscope
        private static final DriverButtons kIntakeButton = DriverButtons.B; //3 Button, run intake
        private static final DriverButtons kOuttakeButton = DriverButtons.Y; //4 Button, run outtake

        private static final int kXTranslationAxis = 0;
        private static final int kYTranslationAxis = 1;
        private static final int kRotationAxis = 2;

        //TODO: Tune curves to driver preference
        private static final ControlCurve kXTranslationCurve = new ControlCurve(0.7,0,0.5,0.1);
        private static final ControlCurve kYTranslationCurve = new ControlCurve(0.7,0,0.5,0.1);
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

        public static POVButton getAlignForwardPOV(){
            return new POVButton(kJoystick, 0);
        }

        public static POVButton getAlignBackPOV(){
            return new POVButton(kJoystick, 180);
        }

        public static POVButton getAlignLeftPOV(){
            return new POVButton(kJoystick, 270);
        }

        public static POVButton getAlignRightPOV(){
            return new POVButton(kJoystick, 90);
        }

        public static JoystickButton getOrientationButton(){
            kOrientationButton.setButtonName("Orientation Button");
            return new JoystickButton(kJoystick, kOrientationButton.getButtonID());
        }
        public static JoystickButton getZeroButton(){
            kZeroButton.setButtonName("Zero Button");
            return new JoystickButton(kJoystick, kZeroButton.getButtonID());
        }
        public static JoystickButton getIntakeButton(){
            kIntakeButton.setButtonName("Intake Button");
            return new JoystickButton(kJoystick, kIntakeButton.getButtonID());
        }
        public static JoystickButton getOuttakeButton(){
            kOuttakeButton.setButtonName("Outtake Button");
            return new JoystickButton(kJoystick, kOuttakeButton.getButtonID());
        }
    }

    public static final class Operator{
        public enum OperatorButtons {
            X (1, null),
            A (2, null),
            B (3, null),
            Y (4, null),
            LB (5, null),
            RB (6, null),
            LT (7, null),
            RT (8, null),
            Back (9, null),
            Start (10, null),
            LJ (11, null),
            RJ (12, null);
            
            private final int buttonID; 
            private String buttonName;
          
            OperatorButtons(int ID, String N) {
              this.buttonID = ID;
              this.buttonName = N;
            }
          
            private int getButtonID(){
              return this.buttonID;
            };

            private String getButtonName(){
                return this.buttonName;
            }

            private void setButtonName(String name){
                this.buttonName = name;
            }
        };
    }

    public static void putControllerButtons(){
        ShuffleboardLayout sbDriverButtons = Shuffleboard.getTab("Controller Buttons")
        .getLayout("Driver Buttons", BuiltInLayouts.kList)
        .withSize(2, 5)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;

        ShuffleboardLayout sbOperatorButtons = Shuffleboard.getTab("Controller Buttons")
        .getLayout("Operator Buttons", BuiltInLayouts.kList)
        .withSize(2, 5)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for Variables;

        for (Driver.DriverButtons button : Driver.DriverButtons.values()) 
        sbDriverButtons.add(String.valueOf(button.getButtonID()), "Button " + button.toString() + ": " + button.getButtonName());
        
        for (Operator.OperatorButtons button : Operator.OperatorButtons.values()) 
        sbOperatorButtons.add(String.valueOf(button.getButtonID()+12), "Button " + button.toString() + ": " + button.getButtonName());
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
