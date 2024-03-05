// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs;
import frc.robot.classes.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class OrbitalTarget extends Command {

  private final Chassis chassis;
  private final PoseEstimator poseEstimator;

  private Translation2d speakerPose;

  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotController;
  private final double rotationFFCoefficient;

  private Rotation2d goalRotation;

  private Supplier<Measure<Distance>> orbitDistance;

  public OrbitalTarget(
      Chassis chassis,
      Supplier<ChassisSpeeds> speedsSupplier,
      PIDConstants translationPID,
      PIDConstants rotationPID,
      Structs.MotionLimits motionLimits,
      PoseEstimator poseEstimator,
      Supplier<Measure<Distance>> orbitDistance,
      double rotationFFCoefficient) {

    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.speedsSupplier = speedsSupplier;
    this.orbitDistance = orbitDistance;
    this.rotationFFCoefficient = rotationFFCoefficient;

    Constraints constraints = new Constraints(motionLimits.maxSpeed, motionLimits.maxAcceleration);
    xController =
        new ProfiledPIDController(
            translationPID.kP, translationPID.kI, translationPID.kD, constraints);
    yController =
        new ProfiledPIDController(
            translationPID.kP, translationPID.kI, translationPID.kD, constraints);
    rotController =
        new ProfiledPIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD, constraints);

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = poseEstimator.getFusedPose();
    Transform2d vel = poseEstimator.getEstimatedVel();
    xController.reset(robotPose.getX(), vel.getX());
    yController.reset(robotPose.getY(), vel.getY());
    rotController.reset(robotPose.getRotation().getRadians(), vel.getRotation().getRadians());
    speakerPose =
        DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get() == Alliance.Red
                ? Constants.RED_SPEAKER_POSE
                : Constants.BLUE_SPEAKER_POSE)
            : Constants.BLUE_SPEAKER_POSE;
  }

  @Override
  public void execute() {
    Pose2d robotPose = poseEstimator.getFusedPose();
    Translation2d speakerPose = this.speakerPose.get();

    Translation2d goalPosition = robotPose.getTranslation().minus(speakerPose);
    /*
     * The goal rotation is the angle between the predicted position of the robot and the speaker.
     * Predicted robot position is calculated by current position plus a vector perpendicular to the difference of the robot to the speaker,
     * scaled by the speed of the robot and a constant for tuning
     * Angle is taken from the difference of the speaker position and the predicted robot position
     */
    goalRotation =
        speakerPose
            .minus(
                robotPose
                    .getTranslation()
                    .plus(
                        Util.perpendicular(goalPosition)
                            .times(
                                speedsSupplier.get().vyMetersPerSecond
                                    * 0.02
                                    * rotationFFCoefficient)))
            .getAngle();

    /*  The goal position is normalized and scaled by the orbit distance, then added to the speaker
    position to get a goal position that is radius distance away from the speaker in the direction of the robot*/
    goalPosition = Util.normalize(goalPosition);
    goalPosition = goalPosition.times(orbitDistance.get().in(Meters));
    goalPosition = goalPosition.plus(speakerPose);

    xController.setGoal(goalPosition.getX());
    yController.setGoal(goalPosition.getY());
    rotController.setGoal(goalRotation.getRadians());

    Logger.recordOutput("OrbitGoal", new Pose2d(goalPosition, goalRotation));

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(robotPose.getX()),
                yController.calculate(robotPose.getY()),
                rotController.calculate(robotPose.getRotation().getRadians())),
            robotPose.getRotation());

    speeds.vyMetersPerSecond += speedsSupplier.get().vyMetersPerSecond;

    chassis.driveRobotRelative(speeds);
  }
}
