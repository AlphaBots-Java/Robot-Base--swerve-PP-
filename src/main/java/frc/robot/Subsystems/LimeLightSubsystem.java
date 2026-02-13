package frc.robot.Subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightSubsystem {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-dir");

    public static double DistanceToTarget()
    {
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly horizontal?
        double limelightMountAngleDegrees = 23.0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = 34.0; 
        // distance from the target to the floor
        double goalHeightCM = 112.5; 
        
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        SmartDashboard.putNumber("ty", Math.tan(angleToGoalRadians));



        //calculate distance
        return (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians);
    }




}