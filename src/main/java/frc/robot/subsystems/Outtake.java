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
  SparkMax outtakeMotor1;
  SparkMax outtakeMotor2;
  /** Creates a new Outtake. */
  public Outtake() {
    this.outtakeMotor1 = new SparkMax(OuttakeConstants.outtakeMotor1, MotorType.kBrushless);
    this.outtakeMotor2 = new SparkMax(OuttakeConstants.outtakeMotor2, MotorType.kBrushless);

      // Configure motor controllers (no encoder config needed for SparkMax)
    SparkMaxConfig outtakeMotor1Config = new SparkMaxConfig();
    outtakeMotor1Config.inverted(false);
    outtakeMotor1Config.idleMode(SparkMaxConfig.IdleMode.kCoast);
    outtakeMotor1Config.smartCurrentLimit(40);
    this.outtakeMotor1.configure(outtakeMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig outtakeMotor2Config = new SparkMaxConfig();
    outtakeMotor2Config.inverted(false);
    outtakeMotor2Config.idleMode(SparkMaxConfig.IdleMode.kCoast);
    outtakeMotor2Config.smartCurrentLimit(40);
    outtakeMotor2Config.follow(this.outtakeMotor1);
    this.outtakeMotor2.configure(outtakeMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Runs the outtake motor at the given speed
  public void runOuttake(double speed) {
    outtakeMotor1.set(speed); // speed between -1.0 (reverse) and 1.0 (forward)
}

// Stops the outtake motor
public void stopOuttake() {
  outtakeMotor1.stopMotor();
}

public void openOuttake()
{
  //outtakeMotor1.set(OuttakeConstants.motorSpeed);
}

public void closeOuttake()
{
  //outtakeMotor1.set(-OuttakeConstants.motorSpeed);
}

}
