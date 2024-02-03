// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.classes.BaseDrive;
import frc.robot.classes.ModuleConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.NeoModule;
import frc.robot.subsystems.chassis.PoseEstimator;
import frc.robot.subsystems.chassis.SwerveModule;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

class TestChassisContainer {
  public final Chassis m_chassis;
  private final BaseDrive m_baseDrive;
  public final PoseEstimator m_poseEstimator;
  private PhotonCamera m_ATCamera;
  private final CommandXboxController m_driveController;
  private final CommandXboxController m_sysIDController;
  private final SendableChooser<Command> autoChooser;

  TestChassisContainer() {

    NeoModule frontLeft = makeSwerveModule(1, 2);
    NeoModule frontRight = makeSwerveModule(3, 4);
    NeoModule backLeft = makeSwerveModule(5, 6);
    NeoModule backRight = makeSwerveModule(7, 8);

    SwerveModule[] modules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};

    SwerveDriveKinematics swerveDriveKinematics = getSwerveDriveKinematics();

    m_ATCamera = new PhotonCamera(RobotConstants.AT_CAMERA_NAME);

    m_chassis = new Chassis(modules, swerveDriveKinematics);

    m_poseEstimator = new PoseEstimator(m_chassis, m_ATCamera, RobotConstants.PIGEON_ID);

    m_driveController = new CommandXboxController(0);
    m_sysIDController = new CommandXboxController(5);

    m_baseDrive =
        new BaseDrive(
            m_driveController.getHID(), RobotConstants.MOTION_LIMITS, RobotConstants.RATE_LIMITS);

    PortForwarder.add(5800, "photonvision.local", 5800);

    m_chassis.setDefaultCommand(
        new FieldOrientedDrive(m_chassis, m_poseEstimator, m_baseDrive::calculateChassisSpeeds));

    AutoBuilder.configureHolonomic(
        m_poseEstimator::getFusedPose, // Robot pose supplier
        m_poseEstimator
            ::resetPose, // Method to reset odometry, TODO check if it still works with the command
        m_chassis::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_chassis::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        RobotConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_chassis // Reference to this subsystem to set requirements
        );

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoMode", autoChooser);

    configureBindings();
  }

  private static SwerveDriveKinematics getSwerveDriveKinematics() {
    Translation2d frontLeftLocation =
        new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0);
    Translation2d frontRightLocation =
        new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0);
    Translation2d backLeftLocation =
        new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0);
    Translation2d backRightLocation =
        new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0);

    return new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  }

  private NeoModule makeSwerveModule(int driveId, int steerId) {
    ModuleConfig.ClosedLoopParameters driveClosedLoopParams =
        new ModuleConfig.ClosedLoopParameters(0.1, 0, 0, 1 / RobotConstants.DRIVE_WHEEL_FREESPEED);
    ModuleConfig.ClosedLoopParameters steerClosedLoopParams =
        new ModuleConfig.ClosedLoopParameters(18, 0, 0, 0);
    return new NeoModule(
        new ModuleConfig(
            driveId,
            steerId,
            driveClosedLoopParams,
            steerClosedLoopParams,
            RobotConstants.DRIVE_ENC_TO_METERS,
            RobotConstants.DRIVE_ENC_VEL_TO_METERS_PER_SECOND,
            RobotConstants.STEER_ENC_POS_TO_METERS,
            RobotConstants.STEER_ENC_VEL_TO_METERS,
            RobotConstants.DRIVE_INVERTED,
            RobotConstants.STEER_INVERTED,
            RobotConstants.STEER_ENC_INVERTED,
            RobotConstants.STEER_ENC_PID_MIN,
            RobotConstants.STEER_ENC_PID_MAX,
            RobotConstants.DRIVE_CURRENT_LIMIT,
            RobotConstants.STEER_CURRENT_LIMIT,
            RobotConstants.NOMINAL_VOLTAGE,
            RobotConstants.DRIVE_IDLE,
            RobotConstants.STEER_IDLE));
  }

  private void configureBindings() {
    m_driveController.start().onTrue(m_poseEstimator.resetPose(new Pose2d()));
    m_driveController
        .rightBumper()
        .whileTrue(
            new OrbitalTarget(
                m_chassis,
                m_driveController::getLeftX,
                m_driveController::getLeftY,
                m_driveController::getRightX,
                m_driveController::getLeftTriggerAxis,
                m_driveController::getRightTriggerAxis,
                RobotConstants.TRANSLATION_PID,
                RobotConstants.ROTATION_PID,
                RobotConstants.MOTION_LIMITS.maxSpeed,
                m_poseEstimator));
    m_driveController
        .a()
        .or(m_driveController.b())
        .or(m_driveController.x())
        .or(m_driveController.y())
        .whileTrue(
            new FieldOrientedWithCardinal(
                m_chassis,
                m_poseEstimator,
                () -> {
                  double xCardinal =
                      (m_driveController.y().getAsBoolean() ? 1 : 0)
                          - (m_driveController.a().getAsBoolean() ? 1 : 0);
                  double yCardinal =
                      (m_driveController.b().getAsBoolean() ? 1 : 0)
                          - (m_driveController.x().getAsBoolean() ? 1 : 0);
                  double dir = -Math.atan2(yCardinal, xCardinal);
                  dir = dir < 0 ? dir + 2 * Math.PI : dir; // TODO check if needed

                  Logger.recordOutput("goalCardinal", dir);
                  return dir;
                },
                m_baseDrive::calculateChassisSpeeds,
                RobotConstants.ROTATION_PID,
                RobotConstants.ROTATION_CONSTRAINTS,
                RobotConstants.ROTATION_FF,
                RobotConstants.ROBOT_RADIUS));

    m_sysIDController.a().whileTrue(m_chassis.sysIdQuasistatic(Direction.kForward));
    m_sysIDController.b().whileTrue(m_chassis.sysIdDynamic(Direction.kForward));
    m_sysIDController.x().whileTrue(m_chassis.sysIdQuasistatic(Direction.kReverse));
    m_sysIDController.y().whileTrue(m_chassis.sysIdDynamic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
