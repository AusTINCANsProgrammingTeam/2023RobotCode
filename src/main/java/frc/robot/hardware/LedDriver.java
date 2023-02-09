package frc.robot.hardware;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** REV Robotics Blinkin LED Driver. */
public class LedDriver {
    private final Spark spark;
    //TODO setbounds
    public LedDriver(int channel) {
        spark = new Spark(channel);
    }

    public void setMode(BlinkinLedMode mode) {
        spark.set(mode.value);
    }

    public static enum BlinkinLedMode {
        SOLID_YELLOW(0.69), SOLID_VIOLET(0.91), OFF(0.99);

    private final double value;

        BlinkinLedMode(double value) {
            this.value = value;
        }
    }

// ('CanMan_Left', 16x16px) With original color and blending
 public static String[][] canman = {
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#f9f9f9", "#bebbb1", "#a99457", "#c39e35", "#a3852e", "#a99a6d", "#ebeae9", "#ffffff", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#979384", "#c19d34", "#fece45", "#b59431", "#cba536", "#b99631", "#a99765", "#f5f5f5", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#594c23", "#f2c442", "#ffce46", "#bf9a33", "#f1c440", "#c9a336", "#d0a938", "#9e894c", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#f4f4f4", "#907a38", "#888786", "#ffffff", "#af913d", "#c6a035", "#876e24", "#796320", "#68551b", "#977b28", "#d7af3a", "#baa15c", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#9d8b55", "#6b6041", "#92792b", "#dedede", "#c4a242", "#e9bd3f", "#e4b93d", "#fccc44", "#fece45", "#b89631", "#d0a938", "#ac9043", "#e5e5e5", "#ffffff", "#ffffff", "#ffffff"}, 
{"#afa280", "#b6973c", "#deb43c", "#dfb63f", "#9b7e2a", "#5a4817", "#67541b", "#deb43c", "#ffcf46", "#c7a235", "#d9b03b", "#fccc45", "#ad9654", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#e4e3e1", "#9e8b54", "#dfb94e", "#b89632", "#907426", "#6d581d", "#efc241", "#ffcf46", "#ffcf46", "#eec141", "#cba437", "#e3b940", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#9d8131", "#907526", "#ba9732", "#deb43c", "#ffcf46", "#ffcf46", "#fece45", "#b39230", "#eec141", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#c4a13c", "#a6872c", "#b59230", "#f3c541", "#ffcf46", "#fece45", "#b49230", "#c5a035", "#e0b743", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#b5953b", "#7b6420", "#bc9832", "#ebbe3f", "#ffcf46", "#8f7426", "#af8f2f", "#bb9832", "#988c66", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#8f7735", "#876d23", "#7a6320", "#ddb33c", "#ffcf46", "#d0a938", "#a7882d", "#ae9037", "#e4e4e4", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#bc9b3e", "#f5c743", "#fbcc44", "#fbcb44", "#ffcf46", "#ffcf46", "#c7a136", "#998650", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#dfdede", "#5f522d", "#796320", "#766328", "#7a6730", "#77601f", "#574a22", "#e8e8e8", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#c5c5c5", "#9e8332", "#ba9732", "#696045", "#595447", "#dbb23b", "#a48731", "#f6f6f6", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#c0a24c", "#fece45", "#ebbf40", "#be9b33", "#fbcc45", "#f2c442", "#766123", "#f6f6f6", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}, 
{"#ffffff", "#ffffff", "#ffffff", "#ffffff", "#afa075", "#b89e56", "#a48e4c", "#c6a23a", "#ddb33c", "#d5ad3b", "#89794d", "#fdfdfd", "#ffffff", "#ffffff", "#ffffff", "#ffffff"}
};
}