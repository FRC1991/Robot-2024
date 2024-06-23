package frc.utils;

public class Utils {
    public static double normalize(double input) {
        if(input > 1) {
            return 1;
        } else if(input < -1) {
            return -1;
        } else if(Double.isNaN(input) || Double.isInfinite(input)) {
            return 0;
        }
        return input;
    }
}
