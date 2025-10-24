package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax leader;
    private final SparkMax follower;
    private final Encoder encoder;

    private final DigitalInput lowerLimit;
    private final DigitalInput upperLimit;

    private final PIDController pid;

    private double targetMeters = 0.0;

    // For velocity calculation
    private double previousPosition = 0.0;
    private double previousTime = 0.0;

    public Elevator() {
        // Create motors
        this.leader = new SparkMax(ElevatorConstants.kFrontMotorCANId, SparkLowLevel.MotorType.kBrushless);
        this.follower = new SparkMax(ElevatorConstants.kRearMotorCANId, SparkLowLevel.MotorType.kBrushless);

        // Configure motor controllers (no encoder config needed for SparkMax)
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.inverted(false);
        leaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leaderConfig.smartCurrentLimit(40);
        this.leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.inverted(true);
        followerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        followerConfig.smartCurrentLimit(40);
        followerConfig.follow(this.leader);
        this.follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Create the separate quadrature encoder
        this.encoder = new Encoder(
                ElevatorConstants.kEncoderChannelA,
                ElevatorConstants.kEncoderChannelB);

        // Set distance per pulse (meters per encoder tick)
        this.encoder.setDistancePerPulse(ElevatorConstants.kMetersPerPulse);

        // Limit switches
        this.lowerLimit = new DigitalInput(ElevatorConstants.kLowerLimitDIO);
        this.upperLimit = new DigitalInput(ElevatorConstants.kUpperLimitDIO);

        // PID + feedforward
        this.pid = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
                ElevatorConstants.kElevatorD);
        this.pid.setTolerance(0.05);

        // Initialize velocity calculation variables
        this.previousPosition = 0.0;
        this.previousTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    // Get current elevator position in meters
    public double getPositionMeters() {
        return this.encoder.getDistance();
    }

    // Get current elevator velocity in meters per second (calculated)
    public double getVelocityMetersPerSec() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double currentPosition = getPositionMeters();

        double velocity = (currentPosition - previousPosition) / (currentTime - previousTime);

        // Update for next calculation
        previousPosition = currentPosition;
        previousTime = currentTime;

        return velocity;
    }

    // Open-loop manual control with speed limiting
    public void setPercentOutput(double percent) {
        percent = Math.max(-1.0, Math.min(1.0, percent));

        // Safety limits
        if (this.isAtUpperLimit() && percent > 0.05)
            percent = 0;
        if (this.isAtLowerLimit() && percent < -0.05)
            percent = 0;

        /*
         * / Apply acceleration limiting for smoother manual control
         * double outputChange = percent - previousOutput;
         * if (outputChange > MAX_OUTPUT_CHANGE_PER_CYCLE) {
         * percent = previousOutput + MAX_OUTPUT_CHANGE_PER_CYCLE;
         * } else if (outputChange < -MAX_OUTPUT_CHANGE_PER_CYCLE) {
         * percent = previousOutput - MAX_OUTPUT_CHANGE_PER_CYCLE;
         * }
         */

        this.leader.set(percent);
    }

    // Closed-loop target
    public void setTargetMeters(double meters) {
        this.targetMeters = Math.max(
                ElevatorConstants.kStopPositions[0],
                Math.min(
                        ElevatorConstants.kStopPositions[ElevatorConstants.kStopPositions.length - 1],
                        meters));
    }

    @Override
    public void periodic() {
        double current = this.getPositionMeters();
        double error = this.targetMeters - current;

        double output = 0.0;

        // Three-speed control: fast, slow, hold
        if (error > 3.0) {
            output = 0.3; // Move up fast when far away
        } else if (error > 1.0) {
            output = 0.15; // Slow down as we approach
        } else if (error < -3.0) {
            output = -0.2; // Move down fast when far away
        } else if (error < -1.0) {
            output = -0.1; // Slow down as we approach
        } else {
            output = 0.1; // Hold position (fighting gravity)
        }

        // Safety limits
        if (this.isAtUpperLimit() && output > 0.05)
            output = 0;
        if (this.isAtLowerLimit() && output < -0.05)
            output = 0;

        this.leader.set(output);

        SmartDashboard.putNumber("Elevator/pos", current);
        SmartDashboard.putNumber("Elevator/target", this.targetMeters);
        SmartDashboard.putNumber("Elevator/error", error);
        SmartDashboard.putNumber("Elevator/output", output);
    }

    /*
     * @Override
     * public void periodic() {
     * double current = this.getPositionMeters();
     * double currentVelocity = this.getVelocityMetersPerSec();
     * 
     * this.pid.setSetpoint(this.targetMeters);
     * 
     * double ff = this.feedforward.calculateWithVelocities(this.pid.getSetpoint() -
     * current, 0.0);
     * //double ff = this.feedforward.calculate(this.pid.getSetpoint() - current,
     * 0.0);
     * double pidOut = this.pid.calculate(current, this.targetMeters);
     * 
     * double rawOutput = pidOut + ff;
     * 
     * // Method 1: Limit the maximum closed-loop output
     * double output = Math.max(-ElevatorConstants.kMaxClosedLoopOutput,
     * Math.min(ElevatorConstants.kMaxClosedLoopOutput, rawOutput));
     * 
     * // Method 2: Velocity-based limiting
     * if (Math.abs(currentVelocity) > ElevatorConstants.kMaxVelocityMetersPerSec) {
     * // If going too fast, reduce output in that direction
     * if (currentVelocity > 0 && output > 0) {
     * output = Math.min(output, 0.1); // Allow only small positive output
     * } else if (currentVelocity < 0 && output < 0) {
     * output = Math.max(output, -0.1); // Allow only small negative output
     * }
     * }
     * 
     * // Method 3: Acceleration limiting for closed loop
     * double outputChange = output - previousOutput;
     * if (outputChange > MAX_OUTPUT_CHANGE_PER_CYCLE) {
     * output = previousOutput + MAX_OUTPUT_CHANGE_PER_CYCLE;
     * } else if (outputChange < -MAX_OUTPUT_CHANGE_PER_CYCLE) {
     * output = previousOutput - MAX_OUTPUT_CHANGE_PER_CYCLE;
     * }
     * 
     * // Safety limits
     * if (this.isAtUpperLimit() && output > 0.05) output = 0;
     * if (this.isAtLowerLimit() && output < -0.05) output = 0;
     * 
     * this.leader.set(output);
     * previousOutput = output;
     * 
     * SmartDashboard.putNumber("Elevator/pos", current);
     * SmartDashboard.putNumber("Elevator/target", this.targetMeters);
     * SmartDashboard.putNumber("Elevator/velocity", currentVelocity);
     * SmartDashboard.putNumber("Elevator/rawOutput", rawOutput);
     * SmartDashboard.putNumber("Elevator/output", output);
     * SmartDashboard.putBoolean("Elevator/lowerLimit", this.isAtLowerLimit());
     * SmartDashboard.putBoolean("Elevator/upperLimit", this.isAtUpperLimit());
     * SmartDashboard.putBoolean("Elevator/atTarget", this.pid.atSetpoint());
     * }
     */

    // Check if elevator is at target position
    public boolean atTarget() {
        return this.pid.atSetpoint();
    }

    // Limit switch checks
    public boolean isAtLowerLimit() {
        boolean atLimit = !this.lowerLimit.get();
        if (atLimit) {
            this.zeroEncoder();
        }
        return atLimit;
    }

    public boolean isAtUpperLimit() {
        return !this.upperLimit.get();
    }

    // Stop elevator immediately
    public void stop() {
        this.leader.stopMotor();
    }

    // Manual zeroing (optional, only if you press button 0)
    public void zeroEncoder() {
        this.encoder.reset();
    }
}