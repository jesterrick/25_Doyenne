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

  // Check if we have a valid target
  public boolean hasTarget() {
    return limelightTable.getEntry("tv").getDouble(0) == 1;
  }

  // Get horizontal offset from crosshair to target (-27 to 27 degrees)
  public double getTargetX() {
    return limelightTable.getEntry("tx").getDouble(0);
  }

  // Get vertical offset (-20.5 to 20.5 degrees)
  public double getTargetY() {
    return limelightTable.getEntry("ty").getDouble(0);
  }

  // Get target area (0-100% of image)
  public double getTargetArea() {
    return limelightTable.getEntry("ta").getDouble(0);
  }

  // Set LED mode (0=pipeline, 1=off, 2=blink, 3=on)
  public void setLEDMode(int mode) {
    limelightTable.getEntry("ledMode").setNumber(mode);
  }

  // Set pipeline (0-9)
  public void setPipeline(int pipeline) {
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }
}
