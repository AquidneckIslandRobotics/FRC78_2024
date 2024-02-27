// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.classes.Structs.MotionLimits;
import org.littletonrobotics.junction.Logger;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;

  public SwerveModuleState[] setStates;

  public SwerveModuleState[] getStates;
  public SwerveModulePosition[] getPositions;

  public final SwerveDriveKinematics kinematics;
  private final MotionLimits motionLimits;

  public Chassis(
      SwerveModule[] modules, SwerveDriveKinematics kinematics, MotionLimits motionLimits) {
    // It reads the number of modules from the RobotConstants
    this.modules = modules;
    this.kinematics = kinematics;
    this.motionLimits = motionLimits;

    getStates = new SwerveModuleState[4];
    getPositions = new SwerveModulePosition[4];
  }

  public void setBrake(Boolean y) {
    for (SwerveModule module : modules) {
      module.setBrake(y);
    }
  }

  public SwerveModulePosition[] getPositions() {
    for (int i = 0; i < modules.length; i++) {
      getPositions[i] = modules[i].getPosition();
    }
    return getPositions;
  }

  public SwerveModuleState[] getStates() {
    for (int i = 0; i < modules.length; i++) {
      getStates[i] = modules[i].getState();
    }
    return getStates;
  }

  // There is probably a better way to feed this into the AutoBuilder, but this is
  // simpler for now
  public ChassisSpeeds getRealChassisSpeed() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, motionLimits.maxSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }

    SwerveModuleState[] realStates = {
      modules[0].getRealState(),
      modules[1].getRealState(),
      modules[2].getRealState(),
      modules[3].getRealState()
    };
    SwerveModuleState[] optimizedStates = {
      modules[0].getOptimizedState(),
      modules[1].getOptimizedState(),
      modules[2].getOptimizedState(),
      modules[3].getOptimizedState()
    };

    Logger.recordOutput("Setting States", states);
    Logger.recordOutput("Optimized States", optimizedStates);
    Logger.recordOutput("Real States", realStates);
  }

  private void voltageDrive(Measure<Voltage> voltage) {
    // Iterate through each of the 4 swerve modules
    for (SwerveModule module : modules) {
      // Call the open loop method, which sends a voltage with no feedback to the motors, but holds
      // the wheel at 0 degrees
      module.openLoopDiffDrive(voltage.in(Volts));
    }
  }

  private SysIdRoutine drivetrainRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(null, null, Seconds.of(3)),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotor, this));

  public void logMotor(SysIdRoutineLog log) {
    for (SwerveModule module : modules) {
      // Each motor will write to the log directly
      module.logMotor(log);
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return drivetrainRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return drivetrainRoutine.dynamic(direction);
  }
}
