package frc.robot.constants;

public class ElevatorConstants {
    public static final int kFrontMotorCANId = 9;
    public static final int kRearMotorCANId = 11;
    public static final int kEncoderChannelA = 3;
    public static final int kEncoderChannelB = kEncoderChannelA - 1;

    public static final double kElevatorChainRatio = 10;
    public static final int kMaxElevatorHeight = 0;

    public static final int kElevatorEncoderDIOPort1 = 3;
    public static final int kElevatorEncoderDIOPort2 = 2;

    public static final int kLowerLimitDIO = 0;
    public static final int kUpperLimitDIO = 1;

    // Encoder conversion - you'll need to calculate this based on your setup
    public static final double kMetersPerPulse = 0.01; // Distance per encoder pulse (replace kMetersPerRev)
    public static final double kMaxHeightMeters = 1.2;
    public static final double kMinHeightMeters = 0.0;

    // Speed limits
    public static final double kMaxElevatorSpeed = 0.5; // Increased from 0.01 for manual control
    public static final double kMaxClosedLoopOutput = 0.5; // Limit PID output to 50%
    public static final double kMaxVelocityMetersPerSec = 0.5; // Max velocity for motion profiling

    // Stops (meters from bottom)
    public static final double[] kStopPositions = {
            0.0, // Ground / start
            19.0, // Stop 1
            38.0, // Stop 2
            57.0, // Stop 3
            76.0 // Stop 4
    };

    // PID Constants
    public static final double kElevatorP = 0.5;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.15;

    // Output limits
    public static final double kMaxOutput = 1.0;
    public static final double kMinOutput = -0.2;
}
