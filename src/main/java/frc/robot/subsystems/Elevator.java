// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

  Encoder elevatorEncoder;
  DigitalInput lowerLimitSwitch;
  DigitalInput upperLimitSwitch;
  SparkMax elevatorDriveMotor1;
  SparkMax elevatorDriveMotor2;

  public Elevator() {
    this.elevatorEncoder = new Encoder(ElevatorConstants.encoderElev, ElevatorConstants.encoderElev + 1);
    this.lowerLimitSwitch = new DigitalInput(ElevatorConstants.kLowerLimitSwitchDIOPort);  
    this.upperLimitSwitch = new DigitalInput(ElevatorConstants.kUpperLimitSwitchDIOPort);
    this.elevatorDriveMotor1 = new SparkMax(ElevatorConstants.frontElevMotor, MotorType.kBrushless);
    this.elevatorDriveMotor2 = new SparkMax(ElevatorConstants.rearElevMotor, MotorType.kBrushless);

    // Configure motor controllers (no encoder config needed for SparkMax)
    SparkMaxConfig elevatorDriveMotor1Config = new SparkMaxConfig();
    elevatorDriveMotor1Config.inverted(false);
    elevatorDriveMotor1Config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    elevatorDriveMotor1Config.smartCurrentLimit(40);
    this.elevatorDriveMotor1.configure(elevatorDriveMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig elevatorDriveMotor2Config = new SparkMaxConfig();
    elevatorDriveMotor2Config.inverted(false);
    elevatorDriveMotor2Config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    elevatorDriveMotor2Config.smartCurrentLimit(40);
    elevatorDriveMotor2Config.follow(this.elevatorDriveMotor1);
    this.elevatorDriveMotor2.configure(elevatorDriveMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isUpperLimitSwitchActive() {
    return !this.upperLimitSwitch.get(); // Inverted logic
  }

  public boolean isLowerLimitSwitchActive() {
    return !this.lowerLimitSwitch.get();
  }

  public double getElevatorPosition() {
    this.elevatorEncoder.setDistancePerPulse(.01);
    if (isLowerLimitSwitchActive()) {
      this.elevatorEncoder.reset();
        System.out.println("Lower limit switch triggered. Encoder reset.");
    }
    double position = this.elevatorEncoder.getDistance();
    System.out.println("Current Encoder Position: " + position);
    return position;
  }
}
