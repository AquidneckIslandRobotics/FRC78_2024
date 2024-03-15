// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class Robot extends LoggedRobot {
  private Thread visionThread;
  private PowerDistribution m_pdp;
  private Command m_autonomousCommand;

  private CompetitionRobotContainer m_robotContainer;

  private static final boolean REPLAY_MODE = false;

  @Override
  public void robotInit() {

    SmartDashboard.putData(CommandScheduler.getInstance());
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(1181, "photonvision.local", 1181);
    if (isReal()) {
      DataLogManager.start();
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      m_pdp = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else if (REPLAY_MODE) {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start();
    // CTRE logger
    SignalLogger.setPath("/U/ctre-logs/");
    SignalLogger.start();
    m_robotContainer = new CompetitionRobotContainer();

    // Driver camera

    visionThread =
        new Thread(
            () -> {
              UsbCamera camera = CameraServer.startAutomaticCapture();
              camera.setResolution(640, 480);

              CvSink cvSink = CameraServer.getVideo();
              CvSource outputStream =
                  CameraServer.putVideo(
                      "INTAKE CAMERA", 320, 240); // Can edit camera resolution in ShuffleBoard

              Mat source = new Mat();
              Mat output = new Mat();

              while (!Thread.interrupted()) {
                if (cvSink.grabFrame(source) == 0) {
                  continue;
                }
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
              }
            });
    visionThread.setDaemon(true);
    visionThread.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.m_poseEstimator.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    m_robotContainer.m_chassis.driveRobotRelative(new ChassisSpeeds());
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    m_robotContainer.m_chassis.driveRobotRelative(new ChassisSpeeds());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
