package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI {
    //Operator Interface (OI) class containing all control information
    private static final int kDriverJoystickPort = 0;
    private static final int kOperatorJoystickPort = 1;

    public static final class Driver{
        private static final Joystick kJoystick = new Joystick(OI.kDriverJoystickPort);

        private static final int kOrientationButtonID = 1; //1 Button, Toggle swerve orientation
        private static final int kZeroButtonID = 2; //2 Button, Zero the gyroscope
        private static final int kHomeButtonID = 3; //TODO: Remove this and all associated bindings

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
            return new JoystickButton(kJoystick, kOrientationButtonID);
        }

        public static JoystickButton getZeroButton(){
            return new JoystickButton(kJoystick, kZeroButtonID);
        }

        public static JoystickButton getHomeButton(){
            return new JoystickButton(kJoystick, kHomeButtonID);
        }
    }

    public static final class Operator{
        private static final Joystick kJoystick = new Joystick(OI.kOperatorJoystickPort);

        //Node selection is from operator perspective
        private static final int kAlignMidLeftButtonID = 0; //Right Trigger, align to mid left node
        private static final int kAlignMidRightButtonID = 0; //Left Trigger, align to mid right node
        private static final int kAlignHighLeftButtonID = 0; //Right Bumper, align to high left node
        private static final int kAlignHighRightButtonID = 0; //Left Bumper, align to high right node

        public static JoystickButton getAlignMidLeftButton(){
            return new JoystickButton(kJoystick, kAlignMidLeftButtonID);
        }

        public static JoystickButton getAlignMidRightButton(){
            return new JoystickButton(kJoystick, kAlignMidRightButtonID);
        }

        public static JoystickButton getAlignHighLeftButton(){
            return new JoystickButton(kJoystick, kAlignHighLeftButtonID);
        }

        public static JoystickButton getAlignHighRightButton(){
            return new JoystickButton(kJoystick, kAlignHighRightButtonID);
        }
    }

    public static class ControlCurve{
        private double ySaturation; //Maximum output, in percentage of possible output
        private double yIntercept; //Minimum output, in percentage of saturation
        private double curvature; //Curvature shift between linear and cubic
        private double deadzone; //Range of input that will always return zero output

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
