package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
    //Operator Interface (OI) class containing all control information
    private static final int kDriverJoystickPort = 0;

    public static final class Driver{
        private static final Joystick kJoystick = new Joystick(OI.kDriverJoystickPort);

        private static final int kOrientationButtonID = 1; //Toggle swerve orientation
        private static final int kZeroButtonID = 2; //Zero the gyroscope

        private static final int kXTranslationAxis = 0;
        private static final int kYTranslationAxis = 1;
        private static final int kRotationAxis = 2;

        private static final ControlCurve kXTranslationCurve = new ControlCurve(0.5,0,0,0.1);
        private static final ControlCurve kYTranslationCurve = new ControlCurve(0.5,0,0,0.1);
        private static final ControlCurve kRotationCurve = new ControlCurve(1,0,0,0.1);

        public static Supplier<Double> getXTranslationSupplier(){
            //This axis is inverted
            return () -> kXTranslationCurve.calculate(-kJoystick.getRawAxis(kXTranslationAxis));
        }

        public static Supplier<Double> getYTranslationSupplier(){
            //This axis is inverted
            return () -> kYTranslationCurve.calculate(-kJoystick.getRawAxis(kYTranslationAxis));
        }

        public static Supplier<Double> getRotationSupplier(){
            return () -> kRotationCurve.calculate(kJoystick.getRawAxis(kRotationAxis));
        }

        public static JoystickButton getOrientationButton(){
            return new JoystickButton(kJoystick, kOrientationButtonID);
        }

        public static JoystickButton getZeroButton(){
            return new JoystickButton(kJoystick, kZeroButtonID);
        }
    }

    public static final class Operator{

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
            /* Two equations, separated by a ternary
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
