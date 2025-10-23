// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkMax m_FrontPickupMotor;

  public Intake() {
    this.m_FrontPickupMotor = new SparkMax(IntakeConstants.kIntakeMotorFront, MotorType.kBrushless);

    SparkMaxConfig frontMotorConfig = new SparkMaxConfig();
    frontMotorConfig.inverted(true);
    frontMotorConfig.idleMode(IdleMode.kBrake);
    frontMotorConfig.smartCurrentLimit(40);
    this.m_FrontPickupMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  @Override
  public void periodic() {

  }

  /*
   * This will turn on the Intake motors.
   * Positive will cause the motors to pick up the coral
   * Negative will push the coral our of the intake
   */
  public void engageIntake(double speed) {
    //speed = Math.max(-IntakeConstants.kIntakeMotorSpeed,
    //    Math.min(IntakeConstants.kIntakeMotorSpeed, speed));
    this.m_FrontPickupMotor.set(speed);
  }

  // This will stop the intake motors
  public void stopIntake() {
    this.m_FrontPickupMotor.set(0);
  }
}
