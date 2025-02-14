// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
//import frc.robot.TyRap24Constants.*;
//import frc.robot.TyRap25Constants.*;
import frc.robot.SparkJrConstants.*;

/**
 * The Limelight subsystem is a light that is lime green. If you look at it at a
 * certain angle, you will go blind, so read this code with caution.
 */
public class Limelight extends SubsystemBase {

  private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable(ID.kFrontLimelightName);
  private final NetworkTableEntry targetInViewEntry = limeTable.getEntry("TargetInView");
  private final NetworkTableEntry tplEntry = limeTable.getEntry("pipeline");
  public boolean targetInView;
  private int count = 0;
  double yawAngleDegrees;
  double xDistanceMeters;
  double yDistanceMeters;
  double zDistanceMeters;
  Pose3d pose3D;
  protected int timeSinceValid = 0;
  private LinearFilter filteredYawDegrees = LinearFilter.movingAverage(6);

  public Limelight() {
    System.out.println("-------- Start Limelight\n");

    CameraServer.startAutomaticCapture(new HttpCamera(pickupLimeLightName, pickupLimeLightHttp));
  }

  // This method is encapsulated so it can be overriden for simulation
  protected double[] getTargetPoseCameraSpace() {
    return LimelightHelpers.getTargetPose_CameraSpace(ID.kFrontLimelightName);
  }

  protected Transform2d getTransposeCameraToRobotSpace() {
    return new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
  }

  // Check if limelight is out of field of view
  public boolean isAllZeros(double[] arr) {
    for (int i = 0; i < arr.length; i++) {
      if (arr[i] != 0) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void periodic() {
    targetInView = targetInViewEntry.getDouble(0) >= 1.0;
    SmartDashboard.putString("plType", LimelightHelpers.getCurrentPipelineType(ID.kFrontLimelightName));
    double[] cameraTargetPose = getTargetPoseCameraSpace();
    if (cameraTargetPose.length > 0) {
      if (isAllZeros(cameraTargetPose)) {
        timeSinceValid++;
      } else {
        xDistanceMeters = cameraTargetPose[0];
        yDistanceMeters = cameraTargetPose[1];
        zDistanceMeters = cameraTargetPose[2];
        yawAngleDegrees = cameraTargetPose[4];
        filteredYawDegrees.calculate(yawAngleDegrees);
        timeSinceValid = 0;
      }
    } else {
      timeSinceValid++;
    }

    if (count % 15 == 0) {
      SmartDashboard.putNumber("xDis", xDistanceMeters);
      SmartDashboard.putNumber("yDis", yDistanceMeters);
      SmartDashboard.putNumber("zDis", zDistanceMeters);
      SmartDashboard.putNumber("yaw", yawAngleDegrees);
      SmartDashboard.putNumber("TV", targetInView ? 1 : 0);
      SmartDashboard.putNumber("LLPl", getLimelightPipeline());
    }
    count++;
    if (count == 150000) {
      count = 0;
    }

  }

  public void setLimelightPipeline(int pipeline) {
    System.out.println("Setting Limelight pipeline to " + pipeline);
    limeTable.getEntry("pipeline").setNumber(pipeline);
  }

  public int getLimelightPipeline() {
    return (int) tplEntry.getDouble(-1);
  }

  public double getYawAngleDegrees() {
    return yawAngleDegrees;
  }

  public double getxDistanceMeters() {
    return xDistanceMeters;
  }

  public double getyDistanceMeters() {
    return yDistanceMeters;
  }

  public double getzDistanceMeters() {
    return zDistanceMeters;
  }

  public double getFilteredYawDegrees() {
    return filteredYawDegrees.lastValue();
  }

  public int getTimeSinceValid() {
    return timeSinceValid;
  }

}