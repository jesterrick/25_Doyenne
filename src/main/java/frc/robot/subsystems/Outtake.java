// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OuttakeConstants;

public class Outtake extends SubsystemBase {
  SparkMax kOuttakeMotor1;
  SparkMax kOuttakeMotor2;
  /** Creates a new Outtake. */
  public Outtake() {
    this.kOuttakeMotor1 = new SparkMax(OuttakeConstants.kOuttakeMotor1, MotorType.kBrushless);
    this.kOuttakeMotor2 = new SparkMax(OuttakeConstants.kOuttakeMotor2, MotorType.kBrushless);

      // Configure motor controllers (no encoder config needed for SparkMax)
    SparkMaxConfig kOuttakeMotor1Config = new SparkMaxConfig();
    kOuttakeMotor1Config.inverted(false);
    kOuttakeMotor1Config.idleMode(SparkMaxConfig.IdleMode.kCoast);
    kOuttakeMotor1Config.smartCurrentLimit(40);
    this.kOuttakeMotor1.configure(kOuttakeMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig kOuttakeMotor2Config = new SparkMaxConfig();
    kOuttakeMotor2Config.inverted(false);
    kOuttakeMotor2Config.idleMode(SparkMaxConfig.IdleMode.kCoast);
    kOuttakeMotor2Config.smartCurrentLimit(40);
    kOuttakeMotor2Config.follow(this.kOuttakeMotor1);
    this.kOuttakeMotor2.configure(kOuttakeMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Runs the outtake motor at the given speed
  public void runOuttake(double speed) {
    kOuttakeMotor1.set(speed); // speed between -1.0 (reverse) and 1.0 (forward)
}

// Stops the outtake motor
public void stopOuttake() {
  kOuttakeMotor1.stopMotor();
}

public void openOuttake()
{
  //kOuttakeMotor1.set(OuttakeConstants.kOuttakeMotorSpeed);
}

public void closeOuttake()
{
  //kOuttakeMotor1.set(-OuttakeConstants.kOuttakeMotorSpeed);
}

}
