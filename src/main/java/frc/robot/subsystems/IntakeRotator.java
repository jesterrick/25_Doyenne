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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRotatorConstants;

public class IntakeRotator extends SubsystemBase {
  private final SparkMax m_IntakeRotateMotor;
  private final RelativeEncoder m_RotateMotorEncoder;

  private double targetPosition = IntakeRotatorConstants.kIntakeRotatorMotorUp; // Start up

  /** Creates a new IntakeRotator. */
  public IntakeRotator() {
    this.m_IntakeRotateMotor = new SparkMax(IntakeRotatorConstants.kIntakeMotorRotatorMotor, MotorType.kBrushless);

    SparkMaxConfig intakeRotateConfig = new SparkMaxConfig();
    intakeRotateConfig.inverted(true);
    intakeRotateConfig.idleMode(IdleMode.kBrake);
    intakeRotateConfig.smartCurrentLimit(40);

    this.m_RotateMotorEncoder = m_IntakeRotateMotor.getEncoder();
    EncoderConfig rotateEncoderConfig = new EncoderConfig();
    intakeRotateConfig.encoder.apply(rotateEncoderConfig);
    
    this.m_IntakeRotateMotor.configure(intakeRotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    double currentPos = this.m_RotateMotorEncoder.getPosition();
    double error = targetPosition - currentPos;
    
    double output = 0.0;
    
    // Simple bang-bang control with slow zone
    if (error > 5.0) {
      output = 0.4; // Move down fast
    } else if (error > 1.0) {
      output = 0.15; // Slow down near target
    } else if (error < -5.0) {
      output = -0.4; // Move up fast
    } else if (error < -1.0) {
      output = -0.15; // Slow down near target
    } else {
      output = 0.0; // At target, stop (brake mode will hold it)
    }
    
    // Safety limits
    if (currentPos <= IntakeRotatorConstants.kIntakeRotatorMotorUp && output < 0) {
      output = 0.0;
    }
    if (currentPos >= IntakeRotatorConstants.kIntakeRotatorMotorDown && output > 0) {
      output = 0.0;
    }
    
    this.m_IntakeRotateMotor.set(output);
    
    SmartDashboard.putNumber("Intake/pos", currentPos);
    SmartDashboard.putNumber("Intake/target", targetPosition);
    SmartDashboard.putNumber("Intake/error", error);
    SmartDashboard.putNumber("Intake/output", output);
  }

  // Simple methods to move up or down
  public void moveUp() {
    this.targetPosition = IntakeRotatorConstants.kIntakeRotatorMotorUp;
  }
  
  public void moveDown() {
    this.targetPosition = IntakeRotatorConstants.kIntakeRotatorMotorDown;
  }
  
  // Check if at target (within 1 rotation)
  public boolean atTarget() {
    return Math.abs(this.targetPosition - this.m_RotateMotorEncoder.getPosition()) < 1.0;
  }
  
  public double getPosition() {
    return this.m_RotateMotorEncoder.getPosition();
  }
}