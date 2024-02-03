// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs;
import frc.robot.constants.Constants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class OrbitalTarget extends Command {

  private final Chassis chassis;
  private final PoseEstimator poseEstimator;

  private final Translation2d targetPose;

  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;

  private PIDConstants translationPID;
  private PIDConstants rotationPID;
  private Structs.MotionLimits motionLimits;

  // Target pose in field space for the robot to move to
  private double xTarget;
  private double yTarget;
  private double rotTarget;

  // Essentially the polar coordinates of the robot relative to the target
  private double targetRobotAngle;
  private double orbitDistance;
  private double lateralSpeed;

  public OrbitalTarget(
      Chassis chassis,
      Supplier<ChassisSpeeds> speedsSupplier,
      PIDConstants translationPID,
      PIDConstants rotationPID,
      Structs.MotionLimits motionLimits,
      PoseEstimator poseEstimator) {

    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.translationPID = translationPID;
    this.rotationPID = rotationPID;
    this.motionLimits = motionLimits;
    this.speedsSupplier = speedsSupplier;

    // Might be shorter way of doing this
    if (DriverStation.getAlliance().isPresent()) {
      targetPose =
          DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
              ? Constants.BLUE_ORBIT_POSE
              : Constants.RED_ORBIT_POSE;
    } else {
      targetPose = Constants.BLUE_ORBIT_POSE;
    }

    xController = new PIDController(translationPID.kP, translationPID.kI, translationPID.kD);
    yController = new PIDController(translationPID.kP, translationPID.kI, translationPID.kD);
    rotController = new PIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD);
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Then converts the polar coordinates to field coordinates
    calcTargetDistance();
    Logger.recordOutput(
        "Orbit Goal", new Pose2d(xTarget, yTarget, Rotation2d.fromRadians(rotTarget)));

    // Then uses PID to move the robot to the target pose
    xController.setSetpoint(xTarget);
    yController.setSetpoint(yTarget);
    rotController.setSetpoint(rotTarget);

    chassis.setChassisSpeed =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(poseEstimator.getFusedPose().getX()),
                yController.calculate(poseEstimator.getFusedPose().getY()),
                rotController.calculate(poseEstimator.getFusedPose().getRotation().getRadians())),
            poseEstimator.getFusedPose().getRotation());

    chassis.convertToStates();
    chassis.drive();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setChassisSpeed = new ChassisSpeeds();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void calcTargetDistance() {
    Structs.Vector2 targetToRobot =
        new Structs.Vector2(targetPose, poseEstimator.getFusedPose().getTranslation());
  }
}
