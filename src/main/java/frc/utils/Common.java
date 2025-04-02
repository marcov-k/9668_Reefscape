package frc.utils;

public class Common {

    public static double clamp(double value, double min, double max, double deadband) {
        if (Math.abs(value) < deadband) return 0;
        else return Math.max(min, Math.min(max, value)); }
    
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));}
}
