package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class RotateToAprilTag extends Command {
  private final Chassis chassis;

  /** Creates a new FieldOrientedDrive. */
  public RotateToAprilTag(Chassis chassis) {
    this.chassis = chassis;

    addRequirements(chassis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed = 0.07;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

    double x = tx.getDouble(0.0);
    double errorx = 0 - x;

    double rotation = errorx * rotationSpeed;

    chassis.driveRobotRelative(new ChassisSpeeds(0, 0, rotation));
  }
}
