package frc.robot.constants;

public class VisionConstants {
    public static final int TEAM_NUMBER = 5919;
    public static final String LIMELIGHT_NAME = "limelight";
    // public static final String LIMELIGHT_URL = "http://10.59.19.11:5800";
    public static final String LIMELIGHT_URL = "http://limelight.local:5800";

    // Pipeline numbers
    public static final int DRIVER_PIPELINE = 0;
    public static final int POST_PIPELINE = 1; // Configure this in Limelight web interface

    // Centering PID constants
    public static final double CENTER_KP = 0.04; // Start low and tune up
    public static final double CENTER_MIN_POWER = 0.08; // Minimum to overcome friction
    public static final double CENTER_MAX_POWER = 0.4; // Safety limit
    public static final double CENTER_TOLERANCE = 1.5; // degrees - how centered is "good enough"
}
