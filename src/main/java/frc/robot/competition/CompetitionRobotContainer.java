// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.classes.BaseDrive;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.FieldOrientedWithCardinal;
import frc.robot.commands.VarFeedPrime;
import frc.robot.commands.VarShootPrime;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feedback;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.NeoModule;
import frc.robot.subsystems.chassis.PoseEstimator;
import frc.robot.subsystems.chassis.SwerveModule;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

class CompetitionRobotContainer {
  public final Chassis m_chassis;
  private final BaseDrive m_baseDrive;
  public final PoseEstimator m_poseEstimator;
  private final Intake m_intake;
  private final Elevator m_Elevator;
  public final Shooter m_Shooter;
  private final Wrist m_Wrist;
  private final Feeder m_feeder;
  final Feedback m_feedback;
  private final CommandXboxController m_driveController;
  private final CommandXboxController m_manipController;
  private final CommandXboxController m_testController;
  private final CommandXboxController sysIdController;
  private final SendableChooser<Command> autoChooser;
  private final Command AmpSetUp;

  CompetitionRobotContainer() {

    NeoModule frontLeft =
        new NeoModule(1, 2, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[0]);

    NeoModule frontRight =
        new NeoModule(3, 4, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[1]);

    NeoModule backLeft =
        new NeoModule(5, 6, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[2]);

    NeoModule backRight =
        new NeoModule(7, 8, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[3]);

    SwerveModule[] modules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};

    SwerveDriveKinematics swerveDriveKinematics = getSwerveDriveKinematics();

    m_chassis = new Chassis(modules, swerveDriveKinematics, RobotConstants.MOTION_LIMITS);

    PhotonCamera sternCam = new PhotonCamera(RobotConstants.STERN_CAM_NAME);
    PhotonCamera starboardCam = new PhotonCamera(RobotConstants.STARBOARD_CAM_NAME);
    PhotonCamera portCam = new PhotonCamera(RobotConstants.PORT_CAM_NAME);

    PhotonPoseEstimator sternCamPE =
        new PhotonPoseEstimator(
            Constants.APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            sternCam,
            RobotConstants.STERN_CAM_POSE);

    PhotonPoseEstimator starboardCamPE =
        new PhotonPoseEstimator(
            Constants.APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            starboardCam,
            RobotConstants.STARBOARD_CAM_POSE);
    PhotonPoseEstimator portCamPE =
        new PhotonPoseEstimator(
            Constants.APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            portCam,
            RobotConstants.PORT_CAM_POSE);

    Pigeon2 pigeon = new Pigeon2(RobotConstants.PIGEON_ID);
    m_poseEstimator =
        new PoseEstimator(
            m_chassis,
            swerveDriveKinematics,
            Constants.APRIL_TAG_FIELD_LAYOUT,
            List.of(sternCamPE, starboardCamPE, portCamPE),
            pigeon,
            RobotConstants.STATE_STD_DEVS,
            RobotConstants.VISION_STD_DEVS,
            RobotConstants.SINGLE_TAG_STD_DEVS,
            RobotConstants.MULTI_TAG_STD_DEVS);

    m_driveController = new CommandXboxController(0);
    m_manipController = new CommandXboxController(1);
    m_testController = new CommandXboxController(2);
    // Put on port 5 because we only want to use this during tests
    sysIdController = new CommandXboxController(5);

    m_baseDrive =
        new BaseDrive(
            m_driveController.getHID(), RobotConstants.MOTION_LIMITS, RobotConstants.RATE_LIMITS);

    PortForwarder.add(5800, "photonvision.local", 5800);

    m_chassis.setDefaultCommand(
        new FieldOrientedDrive(m_chassis, m_poseEstimator, m_baseDrive::calculateChassisSpeeds));

    m_intake =
        new Intake(
            RobotConstants.INTAKE_TOP_ID, RobotConstants.INTAKE_BOTTOM_ID,
            RobotConstants.INTAKE_SPEED_IN, RobotConstants.INTAKE_SPEED_OUT);

    m_Elevator = new Elevator();

    m_Shooter = new Shooter(RobotConstants.SHOOTER_CONFIG);

    m_Wrist =
        new Wrist(
            RobotConstants.WRIST_ID, RobotConstants.WRIST_HIGH_LIM, RobotConstants.WRIST_LOW_LIM);

    m_feeder = new Feeder(RobotConstants.FEED_ID);

    m_feedback = new Feedback(RobotConstants.CANDLE_ID);

    AmpSetUp =
        (m_Wrist.setToTargetCmd(19).alongWith(m_Elevator.setToTarget(13.9)).withName("Amp Set Up"));

    NamedCommands.registerCommand("Intake", pickUpNote());
    NamedCommands.registerCommand("StopShooter", m_Shooter.setSpeedCmd(0));

    NamedCommands.registerCommand(
        "StartShooter", m_Shooter.setSpeedCmd(RobotConstants.AUTO_SHOOT_SPEED));
    NamedCommands.registerCommand(
        "Score", Commands.waitSeconds(0.5).andThen(m_feeder.shoot()).withName("Score"));

    NamedCommands.registerCommand("AmpSetUp", AmpSetUp);
    NamedCommands.registerCommand(
        "scoreInAmp", m_feeder.outtake().withTimeout(2).withName("scoreInAmp"));
    NamedCommands.registerCommand("stow", m_Wrist.stow());
    NamedCommands.registerCommand(
        "Target",
        new FieldOrientedWithCardinal(
                m_chassis,
                m_poseEstimator,
                () -> {
                  Translation2d target =
                      DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                          ? Constants.RED_SPEAKER_POSE
                          : Constants.BLUE_SPEAKER_POSE;
                  double angle =
                      target
                              .minus(m_poseEstimator.getFusedPose().getTranslation())
                              .getAngle()
                              .getRadians()
                          + Math.PI;
                  Logger.recordOutput("Aiming angle", angle);
                  //   angle *=
                  //       m_poseEstimator.getEstimatedVel().getY()
                  //           * RobotConstants.SPEAKER_AIM_VEL_COEFF;
                  return angle;
                },
                m_baseDrive::calculateChassisSpeeds,
                RobotConstants.ROTATION_PID,
                RobotConstants.ROTATION_CONSTRAINTS,
                RobotConstants.ROTATION_FF,
                Units.degreesToRadians(5)) // was 2 changed in b80 for wk4
            .withTimeout(1)
            .withName("Target"));
    NamedCommands.registerCommand("StopShooter", m_Shooter.setSpeedCmd(0));
    // NamedCommands.registerCommand(
    //     "DriveToNote",
    //     pickUpNote()
    //         .deadlineWith(new AlignToNote(m_chassis, () -> new ChassisSpeeds(1.5, 0, 0)))
    //         .withTimeout(2)
    //         .withName("Drive to Note"));
    NamedCommands.registerCommand(
        "DriveToNote",
        pickUpNote()
            .deadlineWith(new AlignToNote(m_chassis, () -> new ChassisSpeeds(1.5, 0, 0)))
            .until(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    return m_poseEstimator.getFusedPose().getX()
                        > 8.25 + RobotConstants.CENTER_LINE_MARGIN;
                  } else {
                    return m_poseEstimator.getFusedPose().getX()
                        < 8.25 - RobotConstants.CENTER_LINE_MARGIN;
                  }
                })
            .withName("Drive to Note"));
    NamedCommands.registerCommand("Stow", m_Wrist.stow());
    NamedCommands.registerCommand(
        "VariableShoot",
        new VarShootPrime(
            m_Wrist,
            m_Elevator,
            m_poseEstimator,
            RobotConstants.SHOOT_POINT,
            () -> m_Shooter.getVelocity() * 60,
            RobotConstants.DISTANCE_RANGE,
            RobotConstants.HEIGHT_LENGTH_COEFF,
            RobotConstants.SHOOTER_RPM_TO_MPS,
            RobotConstants.WRIST_HIGH_LIM));

    // Need to add and then to stop the feed and shooter

    AutoBuilder.configureHolonomic(
        m_poseEstimator::getFusedPose, // Robot pose supplier
        m_poseEstimator::resetPose, // Method to reset odometry
        m_chassis::getRealChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
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

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

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

  Command shortRumble(XboxController controller, RumbleType rumbleType) {
    return Commands.startEnd(
            () -> controller.setRumble(rumbleType, 1),
            () -> controller.setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.5)
        .withName("Rumble");
  }

  private void configureBindings() {
    new Trigger(m_feeder::isNoteQueued)
        .whileTrue(m_feedback.noteInCartridge())
        .and(RobotModeTriggers.teleop())
        .onTrue(shortRumble(m_driveController.getHID(), RumbleType.kRightRumble))
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kRightRumble))
        .onFalse(shortRumble(m_driveController.getHID(), RumbleType.kBothRumble));

    // Rumble controllers when target is detected and we don't have a note
    NetworkTableEntry limelightTargetDetected =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    new Trigger(() -> limelightTargetDetected.getDouble(0.0) == 1.0)
        .and(RobotModeTriggers.teleop())
        .and(() -> !m_intake.hasNote())
        .onTrue(shortRumble(m_driveController.getHID(), RumbleType.kLeftRumble))
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kLeftRumble));

    new Trigger(() -> m_Shooter.isAtSpeed(.9))
        .whileTrue(m_feedback.shooterWheelsAtSpeed())
        .and(RobotModeTriggers.teleop())
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kBothRumble));

    new Trigger(() -> m_Elevator.elevIsAtPos())
        .and(RobotModeTriggers.teleop())
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kBothRumble));

    m_driveController
        .rightBumper()
        .whileTrue(
            pickUpNote()
                .deadlineWith(
                    new AlignToNote(m_chassis, () -> new ChassisSpeeds(2, 0, 0))
                        .withName("Auto Note Align")));

    m_driveController
        .leftBumper()
        .whileTrue(
            new FieldOrientedWithCardinal(
                m_chassis,
                m_poseEstimator,
                () -> {
                  Translation2d target =
                      DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                              == DriverStation.Alliance.Red
                          ? Constants.RED_SPEAKER_POSE
                          : Constants.BLUE_SPEAKER_POSE;
                  double angle =
                      target
                              .minus(m_poseEstimator.getFusedPose().getTranslation())
                              .getAngle()
                              .getRadians()
                          + Math.PI;
                  Logger.recordOutput("Aiming angle", angle);
                  return angle;
                },
                m_baseDrive::calculateChassisSpeeds,
                RobotConstants.ROTATION_PID,
                RobotConstants.ROTATION_CONSTRAINTS,
                RobotConstants.ROTATION_FF,
                Units.degreesToRadians(0))); // was zero changed in b80 before wk4

    m_driveController.povDown().whileTrue(m_chassis.lockWheels());

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
                      (m_driveController.getHID().getYButton() ? 1 : 0)
                          - (m_driveController.getHID().getAButton() ? 1 : 0);
                  double yCardinal =
                      (m_driveController.getHID().getBButton() ? 1 : 0)
                          - (m_driveController.getHID().getXButton() ? 1 : 0);
                  double dir = -Math.atan2(yCardinal, xCardinal);
                  dir = dir < 0 ? dir + 2 * Math.PI : dir; // TODO
                  // check
                  // if
                  // needed

                  Logger.recordOutput("goalCardinal", dir);
                  return dir;
                },
                m_baseDrive::calculateChassisSpeeds,
                RobotConstants.ROTATION_PID,
                RobotConstants.ROTATION_CONSTRAINTS,
                RobotConstants.ROTATION_FF,
                0));

    m_driveController
        .rightStick()
        // Disable for now
        .and(() -> false)
        .whileTrue(
            new AlignToPose(
                    m_chassis,
                    Constants.AMP_TRANSFORM,
                    m_poseEstimator,
                    RobotConstants.TRANSLATION_PID,
                    RobotConstants.ROTATION_PID,
                    RobotConstants.MOTION_LIMITS)
                .alongWith(m_chassis.enableAprilTags())
                .andThen(
                    m_Elevator
                        .setToTarget(16.3)
                        .alongWith(
                            new SequentialCommandGroup(
                                m_Wrist.setToTargetCmd(0),
                                new DriveToAprilTag(m_chassis),
                                m_Wrist.setToTargetCmd(23),
                                Commands.waitSeconds(.2),
                                m_feeder.outtake()))))
        .onFalse(m_Wrist.stow().alongWith(m_chassis.enableNoteDetection()));

    // Zero the elevator when the robot leaves disabled mode and has not been zeroed
    RobotModeTriggers.disabled()
        .negate()
        .and(m_Elevator::hasNotBeenZeroed)
        .onTrue(m_Elevator.zeroElevator());

    // We don't actually care about the DS being attached, but the robot is 'disabled' by default,
    // so onTrue never trips
    RobotModeTriggers.disabled()
        .and(DriverStation::isDSAttached)
        .onTrue(
            Commands.runOnce(m_feedback::disabledColorPattern)
                .ignoringDisable(true)
                .withName("LEDs Disabled"));

    m_manipController
        .x()
        .whileTrue(
            new VarFeedPrime(
                    m_Shooter,
                    m_Elevator,
                    m_poseEstimator,
                    RobotConstants.SHOOT_POINT,
                    () -> RobotConstants.WRIST_PLOP_ANGLE,
                    1 / RobotConstants.SHOOTER_RPM_TO_MPS,
                    RobotConstants.STRAIGHT_DIST_COEFF)
                .withName("FlatShot"))
        .onFalse(m_Wrist.setToTargetCmd(RobotConstants.WRIST_HIGH_LIM));

    m_manipController
        .b()
        .whileTrue(
            new VarFeedPrime(
                    m_Shooter,
                    m_Elevator,
                    m_poseEstimator,
                    RobotConstants.SHOOT_POINT,
                    () -> 52,
                    1 / RobotConstants.SHOOTER_RPM_TO_MPS,
                    RobotConstants.HIGH_DIST_COEFF)
                .alongWith(m_Wrist.setToTargetCmd(52)))
        .onFalse(m_Wrist.stow());

    // Where did the old spinup bind go?
    m_manipController
        .leftTrigger(0.5)
        .whileTrue(
            m_Shooter
                .setSpeedCmd(RobotConstants.SHOOTER_VEL)
                .alongWith(
                    new VarShootPrime(
                        m_Wrist,
                        m_Elevator,
                        m_poseEstimator,
                        RobotConstants.SHOOT_POINT,
                        () -> m_Shooter.getVelocity() * 60,
                        RobotConstants.DISTANCE_RANGE,
                        RobotConstants.HEIGHT_LENGTH_COEFF,
                        RobotConstants.SHOOTER_RPM_TO_MPS,
                        RobotConstants.WRIST_HIGH_LIM))
                .withName("Feed"))
        .onFalse(m_Shooter.setSpeedCmd(0).alongWith(m_Wrist.stow()).withName("Feed End"));

    m_testController.x().whileTrue(m_feedback.rainbows());
    m_testController.b().whileTrue(m_feedback.setColor(Color.kBlue));
    m_testController
        .y()
        .whileTrue(
            pickUpNote()
                .deadlineWith(new AlignToNote(m_chassis, () -> new ChassisSpeeds(2, 0, 0)))
                .withTimeout(2));

    // Amp position
    m_manipController
        .y()
        .whileTrue(
            m_Wrist
                .setToTargetCmd(20)
                .alongWith(m_Elevator.setToTarget(16.3))
                .withName("Amp Set-Up"))
        .onFalse(m_Wrist.stow());

    m_manipController.a().whileTrue(m_Elevator.setToTarget(RobotConstants.ELEVATOR_CLIMB_HEIGHT));

    // m_manipController.x().whileTrue(m_Wrist.setToTarget(38)).onFalse(m_Wrist.stow());

    m_manipController.rightBumper().whileTrue(pickUpNote());

    m_manipController.leftBumper().whileTrue(m_feeder.outtake());

    m_manipController.rightTrigger(0.5).whileTrue(m_feeder.shoot());

    m_testController.a().onTrue(m_Wrist.incrementUp());

    m_testController.b().onTrue(m_Wrist.incrementDown());

    // The routine automatically stops the motors at the end of the command
    // sysIdController.a().whileTrue(m_chassis.sysIdQuasistatic(Direction.kForward));
    // sysIdController.b().whileTrue(m_chassis.sysIdDynamic(Direction.kForward));
    // sysIdController.x().whileTrue(m_chassis.sysIdQuasistatic(Direction.kReverse));
    // sysIdController.y().whileTrue(m_chassis.sysIdDynamic(Direction.kReverse));
    sysIdController.a().whileTrue(m_Shooter.sysIdDynamic(Direction.kForward));
    sysIdController.b().whileTrue(m_Shooter.sysIdDynamic(Direction.kReverse));
    sysIdController.x().whileTrue(m_Shooter.sysIdQuasistatic(Direction.kForward));
    sysIdController.y().whileTrue(m_Shooter.sysIdQuasistatic(Direction.kForward));

    RobotModeTriggers.teleop()
        .onTrue(
            m_Elevator
                .enableBrakeMode()
                .andThen(m_Wrist.enableBrakeMode())
                .andThen(m_chassis.enableBrakeMode()));

    RobotModeTriggers.disabled().onTrue(m_Wrist.enableCoastMode());
    RobotModeTriggers.autonomous().onFalse(m_Shooter.setSpeedCmd(0).ignoringDisable(true));
  }

  public Command pickUpNote() {
    return m_feeder
        .intake()
        .deadlineWith(m_intake.intakeCommand(), m_Wrist.setToTargetCmd(55))
        .withName("Pick Up Note");
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (m_feeder.isNoteQueued()) {
      Translation2d target =
          DriverStation.getAlliance().get() == DriverStation.Alliance.Red
              ? Constants.RED_SPEAKER_POSE
              : Constants.BLUE_SPEAKER_POSE;
      Rotation2d angle =
          target
              .minus(m_poseEstimator.getFusedPose().getTranslation())
              .getAngle()
              .plus(Rotation2d.fromDegrees(180));
      Logger.recordOutput("Aiming angle", angle);

      return Optional.of(angle);
    } else {
      return Optional.empty();
    }
  }
}
