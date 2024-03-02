// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.Structs.Range2D;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final TalonFX feedMotor;
  public final TimeOfFlight feedSensor;
  public final double sensorThreshold;

  private final NetworkTableEntry sensorThresholdEntry =
      SmartDashboard.getEntry("feeder/sensorThreshold");

  /** Creates a new Feed. */
  public Feeder(int feedID, int sensorID, Range2D<Integer> TOFRange, double sensorThreshold) {
    feedMotor = new TalonFX(feedID);

    feedMotor.optimizeBusUtilization();

    feedSensor = new TimeOfFlight(sensorID);
    this.sensorThreshold = sensorThreshold;

    sensorThresholdEntry.setDefaultDouble(sensorThreshold);
    sensorThresholdEntry.setPersistent();
    SmartDashboard.putData(this);

    // If no commands are using this subsystem, stop the motor

  }

  public Command setFeed(double speed) {
    return startEnd(() -> feedMotor.set(speed), () -> feedMotor.set(0))
        .withName("feed[" + speed + "]");
  }

  public boolean isNoteQueued() {
    return feedSensor.getRange() <= sensorThresholdEntry.getDouble(sensorThreshold);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("feeder/isNoteQueued", isNoteQueued());
    Logger.recordOutput("Feed Sensor", feedSensor.getRange());
    Logger.recordOutput("Feed Sensor Sigma", feedSensor.getRangeSigma());
  }
}
