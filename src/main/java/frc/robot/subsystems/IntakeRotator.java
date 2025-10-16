// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRotatorConstants;

public class IntakeRotator extends SubsystemBase {
  private final SparkMax m_IntakeRotateMotor;
  private final RelativeEncoder m_RotateMotorEncoder;

  private double targetIntakePosition = 0.0;

  private final PIDController pid;

  /** Creates a new IntakeRotator. */
  public IntakeRotator() {
    this.m_IntakeRotateMotor = new SparkMax(IntakeRotatorConstants.kIntakeMotorRotatorMotor, MotorType.kBrushless);

     SparkMaxConfig intakeRotateConfig = new SparkMaxConfig();
    intakeRotateConfig.inverted(false);
    intakeRotateConfig.idleMode(IdleMode.kBrake);
    intakeRotateConfig.smartCurrentLimit(40);

    this.m_RotateMotorEncoder = m_IntakeRotateMotor.getEncoder();
    EncoderConfig rotateEncoderConfig = new EncoderConfig();

    // Add the encoder configuration to the controller config
    intakeRotateConfig.encoder.apply(rotateEncoderConfig);
    // Configure the controller
    this.m_IntakeRotateMotor.configure(intakeRotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
     // PID controller
     this.pid = new PIDController(IntakeRotatorConstants.kIntakeP, IntakeRotatorConstants.kIntakeI, IntakeRotatorConstants.kIntakeD);
     this.pid.setTolerance(0.01);
  }

  @Override
  public void periodic() {
    double currentPos = this.getAssemblyPosition();

    // PID output to reach target position
    double pidOutput = this.pid.calculate(currentPos, this.targetIntakePosition);

    // Clamp motor output between -1 and 1
    double output = MathUtil.clamp(pidOutput, -1.0, 1.0);

    // Safety check: if we're at the limits, prevent movement in the wrong direction
    if ((currentPos <= IntakeRotatorConstants.kIntakeRotatorMotorUp && output < 0) ||
        (currentPos >= IntakeRotatorConstants.kIntakeRotatorMotorDown && output > 0)) {
      output = 0.0;
    }

    // Move the assembly
    this.m_IntakeRotateMotor.set(output);

    // Debugging
    SmartDashboard.putNumber("Intake/assemblyPos", currentPos);
    SmartDashboard.putNumber("Intake/targetPos", this.targetIntakePosition);
    SmartDashboard.putNumber("Intake/motorOutput", output);
  }

  /*
   * Returns the position of the Intake Assembly
   */
  public double getAssemblyPosition()
  {
    return this.m_RotateMotorEncoder.getPosition();
  }

  /*
   * Tells the assembly motor to move the intake assembly to a specific position 
   * Position is clamped between kIntakeRotatorMotorUp and kIntakeRotatorMotorDown
   */
  public void rotateAssembly(double position)
  {
    // Clamp the target position to the defined limits
    this.targetIntakePosition = MathUtil.clamp(
      position, 
      IntakeRotatorConstants.kIntakeRotatorMotorUp, 
      IntakeRotatorConstants.kIntakeRotatorMotorDown
    );
  }
}