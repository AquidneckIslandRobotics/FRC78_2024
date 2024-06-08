// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.classes.Structs.MotionLimits;
import org.littletonrobotics.junction.Logger;

public class Chassis extends SubsystemBase {

  private final SwerveModule[] modules;

  private final SwerveDriveKinematics kinematics;
  private final MotionLimits motionLimits;

  private NetworkTable table;

  private Tracer tracer = new Tracer();

  private Watchdog watchdog = new Watchdog(0.02, () -> {});

  public Chassis(
      SwerveModule[] modules, SwerveDriveKinematics kinematics, MotionLimits motionLimits) {
    // It reads the number of modules from the RobotConstants
    this.modules = modules;
    this.kinematics = kinematics;
    this.motionLimits = motionLimits;

    SmartDashboard.putData(this);
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData(enableCoastMode());

    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void setBrake(Boolean y) {
    for (SwerveModule module : modules) {
      module.setBrake(y);
    }
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] currentSwervePositions = new SwerveModulePosition[4];
    for (int i = 0; i < modules.length; i++) {
      currentSwervePositions[i] = modules[i].getPosition();
    }
    return currentSwervePositions;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] currentSwerveStates = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      currentSwerveStates[i] = modules[i].getState();
    }
    return currentSwerveStates;
  }

  // There is probably a better way to feed this into the AutoBuilder, but this is
  // simpler for now
  public ChassisSpeeds getRealChassisSpeed() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, motionLimits.maxSpeed);
    watchdog.addEpoch("Setup");

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
    watchdog.addEpoch("Set States");

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
    watchdog.addEpoch("Log");
  }

  public Command lockWheels() {
    return this.startEnd(
        () -> {
          modules[0].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          modules[1].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
          modules[2].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
          modules[3].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        },
        () -> {});
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
          new SysIdRoutine.Config(null, null, Seconds.of(10)),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotor, this));

  public void logMotor(SysIdRoutineLog log) {
    for (SwerveModule module : modules) {
      // Each motor will write to the log directly
      module.logMotor(log);
    }
  }

  public Command enableCoastMode() {
    return Commands.runOnce(
            () -> {
              for (SwerveModule module : modules) {
                module.enableCoastMode();
              }
            })
        .andThen(new PrintCommand("Coast Mode Set On Chassis"))
        .ignoringDisable(true)
        .withName("Enable Chassis Coast");
  }

  public Command enableBrakeMode() {
    return Commands.runOnce(
            () -> {
              for (SwerveModule module : modules) {
                module.enableBrakeMode();
              }
            })
        .andThen(new PrintCommand("Brake Mode Set On Chassis"))
        .ignoringDisable(true)
        .withName("Enable Chassis Brake");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return drivetrainRoutine.quasistatic(direction).withName("sysIDQuasistatic");
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return drivetrainRoutine.dynamic(direction).withName("sysIDDynamic");
  }

  public Command enableAprilTags() {
    return Commands.runOnce(
        () -> {
          table.getEntry("pipeline").setNumber(1);
        });
  }

  public Command enableNoteDetection() {
    return Commands.runOnce(
        () -> {
          table.getEntry("pipeline").setNumber(0);
        });
  }
}
