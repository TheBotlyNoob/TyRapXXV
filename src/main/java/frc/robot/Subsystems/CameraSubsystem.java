// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  UsbCamera usbCamera;
  UsbCamera fisheyeCamera;
  HttpCamera limelightCamera;
  NetworkTableEntry cameraSelection;
  VideoSink server;

  public CameraSubsystem() {
    // Creates UsbCamera and MjpegServer [1] and connects them
    this.usbCamera = CameraServer.startAutomaticCapture("USB camera", 0);
    //this.fisheyeCamera = CameraServer.startAutomaticCapture("Fisheye camera", 1);
    this.fisheyeCamera = null;
    this.limelightCamera = new HttpCamera("limelight", "http://10.3.86.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(this.limelightCamera);
    this.server = CameraServer.getServer();

    this.cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    this.usbCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

    //this.fisheyeCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 120);

    this.setCameraSource(CameraSourceOption.USB_CAMERA);

    this.usbCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //this.fisheyeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    this.limelightCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command setSourceCommand(CameraSourceOption option) {
    return runOnce(
        () -> {
          this.setCameraSource(option);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static enum CameraSourceOption {
    USB_CAMERA,
    FISHEYE,
    LIMELIGHT
  }

  public void setCameraSource(CameraSourceOption option) {
    System.out.printf("SET CAMERA SOURCE: %s\n", option.toString());
    switch(option) {
      case USB_CAMERA: {
        this.server.setSource(this.usbCamera);
        break;
      }
      // case FISHEYE: {
      //   this.server.setSource(this.fisheyeCamera);
      //   break;
      // }
      case LIMELIGHT: {
        this.server.setSource(this.limelightCamera);
        break;
      }
      default: {
        break;
      }
    }
  }
}
