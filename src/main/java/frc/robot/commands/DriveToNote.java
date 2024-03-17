package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class DriveToNote extends Command {
  private final Chassis chassis;

  /** Creates a new FieldOrientedDrive. */
  public DriveToNote(Chassis chassis) {
    this.chassis = chassis;

    addRequirements(chassis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed = 0.07;
    double movementSpeed = 0.07;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double errorx = 0 - x;
    double errory = 0 + y;

    double rotation = errorx * rotationSpeed;
    double movement = errory * movementSpeed;

    chassis.driveRobotRelative(new ChassisSpeeds(movement, 0, rotation));
  }

  @Override
  public boolean isFinished() {
    return false;
    // double limit = 15.0;

    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry ty = table.getEntry("ty");

    // return ty.getDouble(0.0) <= limit;
  }
}
