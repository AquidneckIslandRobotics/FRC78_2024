// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;

public class FieldOrientedDrive extends Command {
  private final Chassis chassis;
  private final Supplier<ChassisSpeeds> speeds;
  private final PoseEstimator poseEstimator;

  /** Set to 180 when on Red */
  private Rotation2d allianceOffset = Rotation2d.fromDegrees(0);

  /** Creates a new FieldOrientedDrive. */
  public FieldOrientedDrive(
      Chassis chassis, PoseEstimator poseEstimator, Supplier<ChassisSpeeds> speeds) {
    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.speeds = speeds;

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      allianceOffset = Rotation2d.fromDegrees(180);
    } else {
      allianceOffset = Rotation2d.fromDegrees(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.get(), poseEstimator.getFusedPose().getRotation().plus(allianceOffset)));
  }
}
