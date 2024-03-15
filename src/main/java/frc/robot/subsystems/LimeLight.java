package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
public double x = tx.getDouble(0.0);
public double y = ty.getDouble(0.0);
public double area = ta.getDouble(0.0);

int periodicDelayer = 0;
final int periodicDelay = 2; // divide 50 /sec by periodicDelay
@Override public void periodic() {
    if (periodicDelayer == 0) {
            periodicDelayer = periodicDelay;
            x = tx.getDouble(0.0); // X = Horizontal axis. With a positive number being to the right and a negative being to the left
            y = ty.getDouble(0.0); // Y = Vertical axis. With positive number being upwards and negative number downwards
            area = ta.getDouble(0.0);
            //post to smart dashboard periodically
            SmartDashboard.putNumber("LimelightX", x);
            SmartDashboard.putNumber("LimelightY", y);
            SmartDashboard.putNumber("LimelightArea", area);
        }
    else --periodicDelayer;
    }
}
