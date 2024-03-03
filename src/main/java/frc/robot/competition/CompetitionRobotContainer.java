// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.classes.BaseDrive;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.FieldOrientedWithCardinal;
import frc.robot.commands.OrbitalTarget;
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
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

class CompetitionRobotContainer {
  public final Chassis m_chassis;
  private final BaseDrive m_baseDrive;
  public final PoseEstimator m_poseEstimator;
  private final PhotonCamera m_ATCamera;
  private final Intake m_intake;
  private final Elevator m_Elevator;
  private final Shooter m_Shooter;
  private final Wrist m_Wrist;
  private final Feeder m_feeder;
  final Feedback m_feedback;
  private final CommandXboxController m_driveController;
  private final CommandXboxController m_manipController;
  private final CommandXboxController m_testController;
  private final CommandXboxController sysIdController;
  private final SendableChooser<Command> autoChooser;
  private final Command pickUpNote;
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

    m_ATCamera = new PhotonCamera(RobotConstants.AT_CAMERA_NAME);

    m_chassis = new Chassis(modules, swerveDriveKinematics, RobotConstants.MOTION_LIMITS);

    m_poseEstimator =
        new PoseEstimator(
            m_chassis,
            m_ATCamera,
            RobotConstants.CAM1_OFFSET,
            RobotConstants.PIGEON_ID,
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

    pickUpNote =
        m_intake
            .intakeCommand()
            .alongWith(m_Wrist.setToTargetCmd(55)) // new intake angle (stow is 55 as well, but
            // calling it here due to auto
            // using other positions)
            .deadlineWith(m_feeder.intake());
    AmpSetUp = (m_Wrist.setToTargetCmd(19).alongWith(m_Elevator.setToTarget(13.9)));

    NamedCommands.registerCommand("Intake", pickUpNote);
    NamedCommands.registerCommand(
        "ScoreFromW2",
        m_Shooter
            .setSpeed(RobotConstants.AUTO_SHOOT_SPEED)
            .alongWith(m_Wrist.setToTargetCmd(RobotConstants.WRIST_W2_TARGET))
            .andThen(Commands.waitUntil(m_Wrist::isAtTarget).withTimeout(1)));
    NamedCommands.registerCommand(
        "StartShooter", m_Shooter.setSpeed(RobotConstants.AUTO_SHOOT_SPEED));
    NamedCommands.registerCommand("Score", m_feeder.shoot());
    NamedCommands.registerCommand("AmpSetUp", AmpSetUp);
    NamedCommands.registerCommand("scoreInAmp", m_feeder.outtake().withTimeout(2));
    NamedCommands.registerCommand("stow", m_Wrist.stow());

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

  Command shortRumble(XboxController controller) {
    return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1))
        .andThen(new WaitCommand(.5))
        .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
  }

  private void configureBindings() {
    new Trigger(m_feeder::isNoteQueued)
        .onTrue(shortRumble(m_driveController.getHID()))
        .onTrue(m_feedback.noteInCartridge())
        .onFalse(shortRumble(m_driveController.getHID()));
    new Trigger(() -> m_Shooter.isAtSpeed(.9))
        .onTrue(shortRumble(m_manipController.getHID()))
        .onTrue(m_feedback.shooterWheelsAtSpeed());
    new Trigger(() -> m_intake.hasNote())
        .onTrue(m_feedback.intakeCurrentSpike())
        .onFalse(m_feedback.turnOffLEDs());
    m_driveController
        .start()
        .onTrue(new InstantCommand(() -> m_poseEstimator.resetPose(new Pose2d())));

    m_driveController
        .rightBumper()
        .whileTrue(
            new RunCommand(
                () -> m_chassis.driveRobotRelative(m_baseDrive.calculateChassisSpeeds()),
                m_chassis));

    m_driveController
        .pov(180)
        .whileTrue(
            new OrbitalTarget(
                m_chassis,
                m_baseDrive::calculateChassisSpeeds,
                RobotConstants.TRANSLATION_PID,
                RobotConstants.ROTATION_PID,
                RobotConstants.MOTION_LIMITS,
                m_poseEstimator,
                () -> Constants.ORBIT_RADIUS,
                RobotConstants.ORBITAL_FF_CONSTANT));
    m_driveController
        .leftBumper()
        .whileTrue(
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
                RobotConstants.ROTATION_FF));
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
                RobotConstants.ROTATION_FF));

    // Zero the elevator when the robot leaves disabled mode and has not been zeroed
    RobotModeTriggers.disabled()
        .negate()
        .and(m_Elevator::hasNotBeenZeroed)
        .onTrue(m_Elevator.zeroElevator());

    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(() -> m_feedback.disabledColorPattern()).ignoringDisable(true));

    m_manipController
        .leftTrigger(0.5)
        .whileTrue(m_Shooter.setSpeed(5000))
        .whileFalse(m_Shooter.setSpeed(0));
    // TODO switch the variable code onto left trigger

    // Sets elevator and wrist to Amp score position
    // m_manipController
    // .y()
    // .whileTrue(m_Wrist.setToTarget(19).alongWith(m_Elevator.setToTarget(13.9)))
    // .onFalse(m_Wrist.stow());

    new Trigger(m_feeder::isNoteQueued)
        .onTrue(
            Commands.runOnce(
                () ->
                    m_Wrist.setDefaultCommand(
                        new VarShootPrime(
                            m_Wrist,
                            m_Elevator,
                            m_poseEstimator,
                            RobotConstants.SHOOT_POINT,
                            RobotConstants.SHOOTER_VEL,
                            RobotConstants.DISTANCE_RANGE,
                            RobotConstants.HEIGHT_LENGTH_COEFF,
                            RobotConstants.SHOOTER_RPM_TO_MPS))))
        .onFalse(
            Commands.runOnce(
                () ->
                    m_Wrist.setDefaultCommand(
                        m_Wrist.setToTargetCmd(RobotConstants.WRIST_HIGH_LIM))));

    // Where did the old spinup bind go?
    m_manipController
        .leftTrigger(0.5)
        .whileTrue(m_Shooter.setSpeed(RobotConstants.SHOOTER_VEL))
        .onFalse(m_Shooter.setSpeed(0));

    m_testController.x().whileTrue(m_feedback.rainbows());
    m_testController.b().whileTrue(m_feedback.setColor(Color.kBlue));

    m_manipController
        .y()
        .whileTrue(
            m_Wrist
                .setToTargetCmd(19)
                .alongWith(m_Elevator.setToTarget(13.9))); // Sets to AMP // sets to STOW
    m_manipController.a().whileTrue(m_Elevator.setToTarget(RobotConstants.ELEVATOR_CLIMB_HEIGHT));

    m_manipController.b().whileTrue(m_Elevator.setToTarget(2));

    // m_manipController.x().whileTrue(m_Wrist.setToTarget(38)).onFalse(m_Wrist.stow());

    m_manipController.rightBumper().whileTrue(pickUpNote);

    m_manipController.leftBumper().whileTrue(m_feeder.intake());

    m_manipController.rightTrigger(0.5).whileTrue(m_feeder.shoot());

    m_testController.a().onTrue(m_Wrist.incrementUp());

    m_testController.b().onTrue(m_Wrist.incrementDown());

    // The routine automatically stops the motors at the end of the command
    sysIdController.a().whileTrue(m_chassis.sysIdQuasistatic(Direction.kForward));
    sysIdController.b().whileTrue(m_chassis.sysIdDynamic(Direction.kForward));
    sysIdController.x().whileTrue(m_chassis.sysIdQuasistatic(Direction.kReverse));
    sysIdController.y().whileTrue(m_chassis.sysIdDynamic(Direction.kReverse));

    RobotModeTriggers.teleop()
        .onTrue(
            m_Elevator
                .enableBrakeMode()
                .andThen(m_Wrist.enableBrakeMode())
                .andThen(m_chassis.enableBrakeMode()));

    RobotModeTriggers.disabled()
        .onTrue(m_Wrist.enableCoastMode().andThen(m_chassis.enableCoastMode()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
