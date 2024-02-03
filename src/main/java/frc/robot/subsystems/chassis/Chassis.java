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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;

  public ChassisSpeeds setChassisSpeed;
  public SwerveModuleState[] setStates;

  public ChassisSpeeds getChassisSpeed;
  public SwerveModuleState[] getStates;
  public SwerveModulePosition[] getPositions;

  public final SwerveDriveKinematics kinematics;

  public Chassis(SwerveModule[] modules, SwerveDriveKinematics kinematics) {
    // It reads the number of modules from the RobotConstants
    this.modules = modules;
    this.kinematics = kinematics;

    getChassisSpeed = new ChassisSpeeds();
    setChassisSpeed = new ChassisSpeeds();
    getStates = new SwerveModuleState[4];
    getPositions = new SwerveModulePosition[4];
  }

  public void initializeModules() {
    // This is an example of how we will perform operations on all modules
    for (SwerveModule module : modules) {
      module.initialize();
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
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public void convertToStates() {
    setStates = kinematics.toSwerveModuleStates(setChassisSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setChassisSpeed = speeds;
    convertToStates();
    drive();
  }

  public void drive() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(setStates[i]);
      SmartDashboard.putNumber(i + " Rot", setStates[i].angle.getRotations());
    }

    Logger.recordOutput("ModuleSet", setStates);
  }

  private void voltageDrive(Measure<Voltage> voltage) {
    // Iterate through each of the 4 swerve modules
    for (SwerveModule module : modules) {
      // Call the open loop method, which sends a voltage with no feedback to the motors, but holds
      // the wheel at 0 degrees
      module.openLoopDiffDrive(voltage.in(Volts), 0);
    }
    modules[0].openLoopDiffDrive(voltage.in(Volts), 0.375);
    modules[1].openLoopDiffDrive(voltage.in(Volts), 0.125);
    modules[2].openLoopDiffDrive(voltage.in(Volts), -0.375);
    modules[3].openLoopDiffDrive(voltage.in(Volts), -0.125);
  }

  private SysIdRoutine drivetrainRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(null, null, Seconds.of(3)),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotor, this));

  public void logMotor(SysIdRoutineLog log) {
    int i = 0;
    for (SwerveModule module : modules) {
      // Each motor will write to the log directly
      module.logMotor(log);
      // log.motor("motor#" + i);
      i++;
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return drivetrainRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return drivetrainRoutine.dynamic(direction);
  }
}
