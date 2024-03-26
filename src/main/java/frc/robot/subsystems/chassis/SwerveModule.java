// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

/**
 * This is the parent interface for a SwerveModule. It is used to define the methods that all
 * SwerveModules must implement. Recourse for this can be found in the repository wiki,
 * https://github.com/AquidneckIslandRobotics/78Offseason2023/wiki/Concepts
 */
public interface SwerveModule {

  void initialize();

  void setBrake(Boolean y);

  double getDriveVelocity(); // Get current drive velocity from  (m/s)

  double getDrivePosition(); // Get the relative position of the angle motor's encoder

  Rotation2d getSteerPosition(); // Get the absolute position of the magnetic encoder

  SwerveModuleState getOptimizedState();

  SwerveModuleState getRealState();

  SwerveModuleState getState();

  SwerveModulePosition getPosition();

  void setState(SwerveModuleState state);

  /** Runs the drive motor at a set voltage, while keeping the steer angle at 0 degrees */
  void openLoopDiffDrive(double voltage);

  /** Logs the motor position, velocity, and voltage data for SysId */
  void logMotor(SysIdRoutineLog log);

  void enableBrakeMode();

  void enableCoastMode();
}
