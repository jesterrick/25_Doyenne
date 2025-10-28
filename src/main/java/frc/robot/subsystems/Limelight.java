// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cscore.HttpCamera;


public class Limelight extends SubsystemBase {
  private final NetworkTable limelightTable;
  /** Creates a new Limelight. */
  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
    HttpCamera limelightCamera = new HttpCamera(VisionConstants.LIMELIGHT_NAME, VisionConstants.LIMELIGHT_URL);
    CameraServer.startAutomaticCapture(limelightCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
