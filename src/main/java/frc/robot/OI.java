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
    private static final int kOperatorJoystickPort = 1;

    public static final class Driver{
        private static enum Button {
            button1 (1),
            button2 (2),
            button3 (3),
            button4 (4),
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
        
          private static final Button kOrientationButton = Button.button1; //1, Toggle swerve orientation
          private static final Button kZeroButton = Button.Start; //3, Zero the gyroscope
          private static final Button kOuttakeButton = Button.LB; //Right Bumper, run outtake
          private static final Button kIntakeButton = Button.LT; //Right Trigger, run intake
          private static final Button kAlignForwardButton = Button.button4; //4, Align forwards
          private static final Button kAlignBackwardButton = Button.button2; //2, Align backwards
          private static final Button kArmHighTrigger = Button.RB;
          private static final Button kArmMidTrigger = Button.RT;
          private static final Button kFloorIntake = Button.button3;
        
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

        public static JoystickButton getIntakeButton(){
            kIntakeButton.setButtonAction("Intake");
            return new JoystickButton(kJoystick, kIntakeButton.getButtonID());
        }

        public static JoystickButton getOuttakeButton(){
            kOuttakeButton.setButtonAction("Outtake");
            return new JoystickButton(kJoystick, kOuttakeButton.getButtonID());
        }

        public static JoystickButton getArmHighTrigger(){
            kArmHighTrigger.setButtonAction("Arm High");
            return new JoystickButton(kJoystick, kArmHighTrigger.getButtonID());
        }

        public static JoystickButton getArmMidTrigger(){
            kArmMidTrigger.setButtonAction("Arm Mid");
            return new JoystickButton(kJoystick, kArmMidTrigger.getButtonID());
        }

        public static JoystickButton getFloorIntakeButton(){
            kFloorIntake.setButtonAction("Floor Intake");
            return new JoystickButton(kJoystick, kFloorIntake.getButtonID());
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
            RJ (12),  // Right Joystick Button
            POVUP (0),
            POVDOWN (180),
            POVLEFT (270),
            POVRIGHT (90);
            
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

        private static final Joystick kJoystick = new Joystick(OI.kOperatorJoystickPort);

        private static final Button kBuddyBalanceActivateButton = Button.B; //Activates buddy balance
        private static final Button kDownBuddyBalanceButton = Button.POVDOWN; // Lowers buddy balance lift
        private static final Button kUpBuddyBalanceButton = Button.POVUP; // Raises buddy balance lift
        private static final Button kYellowCargo = Button.Y;
        private static final Button kPurpleCargo = Button.X;

        private static final int kArmElbowAxis = 0;
        private static final int kArmBaseAxis = 2;

        //TODO: Tune curves to driver preference
        private static final ControlCurve kArmElbowCurve = new ControlCurve(0.65,0.05,1,0.1);
        private static final ControlCurve kArmBaseCurve = new ControlCurve(0.65,0.05,1,0.1);
 
        public static Supplier<Double> getArmElbowSupplier(){
            //This axis is inverted
            return () -> kArmElbowCurve.calculate(-kJoystick.getRawAxis(kArmElbowAxis));
        }

        public static Supplier<Double> getArmBaseSupplier(){
            //This axis is inverted
            return () -> kArmBaseCurve.calculate(-kJoystick.getRawAxis(kArmBaseAxis));
        }

        public static JoystickButton getActivateBuddyBalanceButton() {
            kBuddyBalanceActivateButton.setButtonAction("Activate buddy balance");
            return new JoystickButton(kJoystick, kBuddyBalanceActivateButton.getButtonID()); // This button must be held in order for the buddy balance to function
        }

        public static POVButton getDownBuddyBalanceButton() {
            kDownBuddyBalanceButton.setButtonAction("Lower buddy balance");
            return new POVButton(kJoystick, kDownBuddyBalanceButton.getButtonID()); // This button will deploy the buddy balance if the confirm button is also held, and also move the lift to the deployed position if it was in the balanced position
        }

        public static POVButton getUpBuddyBalanceButton() {
            kUpBuddyBalanceButton.setButtonAction("Raise buddy balance");
            return new POVButton(kJoystick, kUpBuddyBalanceButton.getButtonID()); // This button will move the lift to the balanced position if it was in the deployed position
        }

        public static JoystickButton getYellowCargoButton() {
            kYellowCargo.setButtonAction("Yellow Cargo Lights");
            return new JoystickButton(kJoystick, kYellowCargo.getButtonID());
        }

        public static JoystickButton getPurpleCargoButton() {
            kYellowCargo.setButtonAction("Purple Cargo Lights");
            return new JoystickButton(kJoystick, kPurpleCargo.getButtonID());
        }
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
