// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.utilities.Util;
//import static frc.robot.utilities.Util.round2;
/**
 * The Limelight subsystem is a light that is lime green. If you look at it at a
 * certain angle, you will go blind, so read this code with caution.
 */
public class Limelight extends SubsystemBase {

  
  private static String pickupLimeLightName = "limelight-c";
  //private static String pickupLimeLightHttp = "http://10.3.86.13";
  private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable(pickupLimeLightName);
  private final NetworkTableEntry txEntry = limeTable.getEntry("tx");
  private final NetworkTableEntry tyEntry = limeTable.getEntry("ty");
  private final NetworkTableEntry tvEntry = limeTable.getEntry("tv");
  private final NetworkTableEntry taEntry = limeTable.getEntry("ta");
  private final NetworkTableEntry tplEntry = limeTable.getEntry("pipeline");
  public double tx;
  public double ty;
  public boolean tv;
  public double ta;
  public double previousAngle = Double.MAX_VALUE;
  public double previousTx = tx;
  public double deltaTx = 0;
  private int count = 0;


  public Limelight() {
    //Util.logf("-------- Start Limelight %s\n", Robot.alliance);
    System.out.println("-------- Start Limelight\n");
  }
  @Override
  public void periodic() {
    //tx = round2(txEntry.getDouble(0));
    //ty = round2(tyEntry.getDouble(0));
    //tv = tvEntry.getDouble(0) == 1;
    //ta = round2(taEntry.getDouble(0));
    tx = txEntry.getDouble(0);
    ty = tyEntry.getDouble(0);
    tv = tvEntry.getDouble(0) == 1;
    ta = taEntry.getDouble(0);
    deltaTx = Math.abs(tx - previousTx);
    previousTx = tx;
    if (count % 15 == 0) {
      SmartDashboard.putNumber("TX", tx);
      SmartDashboard.putNumber("TY", ty);
      SmartDashboard.putNumber("TA", ta);
      SmartDashboard.putNumber("TV", tv ? 1 : 0);
      SmartDashboard.putBoolean("TVB", tv);
      SmartDashboard.putNumber("LLPl", getLimelightPipeline());
    }
    count++;
    if (count == 150000)
    {
      count = 0;
    }
  }

  public void setLimelightPipeline(int pipeline) {
     //int prevPipeline = getLimelightPipeline();
    //Util.logf("-----------  Set Limelight pipeline robot alliance:%s new:%d prev:%d\n", Robot.alliance,  pipeline,
    //    prevPipeline);
    System.out.println("Setting Limelight pipeline to " + pipeline);
    limeTable.getEntry("pipeline").setNumber(pipeline);
  }
  public int getLimelightPipeline() {
    return (int) tplEntry.getDouble(-1);
  }

}