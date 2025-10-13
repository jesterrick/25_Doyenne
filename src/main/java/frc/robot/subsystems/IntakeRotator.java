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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRotatorConstants;

public class IntakeRotator extends SubsystemBase {
  private final SparkMax m_IntakeRotateMotor;
  private final RelativeEncoder m_RotateMotorEncoder;

  private double targetIntakePosition = 0.0;

  private final PIDController pid;
  private final SimpleMotorFeedforward feedforward;

  /** Creates a new IntakeRotator. */
  public IntakeRotator() {
    this.m_IntakeRotateMotor = new SparkMax(IntakeRotatorConstants.kRotateAssemblyMotorId, MotorType.kBrushless);

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

    
     // PID + feedforward
     this.pid = new PIDController(IntakeRotatorConstants.kIntakeP, IntakeRotatorConstants.kIntakeI, IntakeRotatorConstants.kIntakeD);
     this.pid.setTolerance(0.01);

     this.feedforward = new SimpleMotorFeedforward(
      IntakeRotatorConstants.kIntakeS,
      IntakeRotatorConstants.kIntakeV,
      IntakeRotatorConstants.kIntakeA);
  }

  @Override
  public void periodic() {
    double currentPos = this.getAssemblyPosition();

    // Set PID setpoint
    // defines where the assembly needs to go
    this.pid.setSetpoint(this.targetIntakePosition);

    // PID output
    // this is important so we don't more the assembly too fast
    double pidOutput = this.pid.calculate(currentPos, this.targetIntakePosition);

    // Feedforward
    // helps with smooth movements
    double ff = this.feedforward.calculate(this.pid.getSetpoint() - currentPos, 0.0);

    // Total motor output, clamped between -1 and 1
    // using PID and feed forward, calculate the speed and be smooth
    double output = Math.max(-1.0, Math.min(1.0, pidOutput + ff));

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
   */
  public void rotateAssembly(double position)
  {
    this.targetIntakePosition = position;
  }
}
