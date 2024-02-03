// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter;
import frc.robot.classes.ModuleConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.Elevator;
import frc.robot.subsystems.chassis.NeoModule;
import frc.robot.subsystems.chassis.SwerveModule;
import org.photonvision.PhotonCamera;

class CompetitionRobotContainer {
  private final Chassis m_chassis;
  private final PhotonCamera m_ATCamera;
  private final Intake m_intake;
  private final Elevator m_Elevator;
  private final Shooter m_shooter;
  private final XboxController m_driveController;
  private final XboxController m_manipController;
  private final CommandXboxController sysIdController;
  private final SendableChooser<Command> autoChooser;

  CompetitionRobotContainer() {

    NeoModule frontLeft = makeSwerveModule(1, 2);
    NeoModule frontRight = makeSwerveModule(3, 4);
    NeoModule backLeft = makeSwerveModule(5, 6);
    NeoModule backRight = makeSwerveModule(7, 8);

    SwerveModule[] modules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};

    SwerveDriveKinematics swerveDriveKinematics = getSwerveDriveKinematics();

    m_ATCamera = new PhotonCamera(RobotConstants.AT_CAMERA_NAME);

    m_chassis = new Chassis(modules, swerveDriveKinematics, RobotConstants.PIGEON_ID, m_ATCamera);

    m_driveController = new XboxController(0);
    m_manipController = new XboxController(1);
    // Put on port 5 because we only want to use this during tests
    sysIdController = new CommandXboxController(5);

    PortForwarder.add(5800, "photonvision.local", 5800);

    m_chassis.setDefaultCommand(
        new Drive(
            m_chassis,
            m_driveController::getLeftY,
            m_driveController::getLeftX,
            m_driveController::getRightX,
            m_driveController::getLeftTriggerAxis,
            m_driveController::getRightTriggerAxis,
            m_driveController::getYButton,
            m_driveController::getBButton,
            m_driveController::getAButton,
            m_driveController::getXButton,
            RobotConstants.MAX_SPEED,
            RobotConstants.MAX_ANGULAR_VELOCITY,
            RobotConstants.ROTATION_PID));

    m_intake =
        new Intake(
            RobotConstants.INTAKE_TOP_ID, RobotConstants.INTAKE_BOTTOM_ID,
            RobotConstants.INTAKE_SPEED_IN, RobotConstants.INTAKE_SPEED_OUT);

    m_Elevator = new Elevator();

    m_shooter = new Shooter(RobotConstants.SHOOTER_TOP_ID, RobotConstants.SHOOTER_BOTTOM_ID);

    AutoBuilder.configureHolonomic(
        m_chassis::getFusedPose, // Robot pose supplier
        m_chassis
            ::resetPose, // Method to reset odometry (will be called if your auto has as starting
        // pose)
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
        m_chassis);

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
        new ModuleConfig.ClosedLoopParameters(0.1, 0, 0, 1);
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
    new Trigger(m_driveController::getStartButton)
        .onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d())));

    new Trigger(m_driveController::getRightBumper)
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
                RobotConstants.MAX_SPEED));

    new Trigger(m_manipController::getYButton).whileTrue(m_Elevator.moveElevatorUp());

    new Trigger(m_manipController::getXButton).whileTrue(m_Elevator.moveElevatorDown());

    new Trigger(m_driveController::getAButton).whileTrue(m_intake.intakeCommand());
    new Trigger(m_driveController::getBButton).whileTrue(m_intake.outtakeCommand());

    new Trigger(m_manipController::getLeftBumper).onTrue(m_shooter.setShooter(90)).onFalse(m_shooter.setShooter(0));

    // The routine automatically stops the motors at the end of the command
    sysIdController.a().whileTrue(m_chassis.sysIdQuasistatic(Direction.kForward));
    sysIdController.b().whileTrue(m_chassis.sysIdDynamic(Direction.kForward));
    sysIdController.x().whileTrue(m_chassis.sysIdQuasistatic(Direction.kReverse));
    sysIdController.y().whileTrue(m_chassis.sysIdDynamic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
