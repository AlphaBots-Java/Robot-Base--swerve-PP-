package frc.robot.Subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class LimeLightSubsystem {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-dir");

    public static boolean hasTarget() {
        NetworkTableEntry tv = table.getEntry("tv");
        return tv.getDouble(0.0) == 1.0; // tv = 1 se um alvo é visível, 0 se não
    }
    public static double getTx() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0.0); // tx = desvio horizontal do alvo em graus
    }
    public static double DistanceToTarget()  
    {
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly horizontal?
        double limelightMountAngleDegrees = 19.5; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = 40.0; 
        // distance from the target to the floor
        double goalHeightCM = 112.5; 
        
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        SmartDashboard.putNumber("ty", Math.tan(angleToGoalRadians));

        //calculate distance
        return (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians);
    }
    public static double calculateSpeed(){
        if (LimeLightSubsystem.DistanceToTarget() < ShooterConstants.kDistanceForHighSpeedMeters){
           return ShooterConstants.kShooterLowSpeedRPM;
        }else{
            return ShooterConstants.kShooterHighSpeedRPM;
        }
    }
    public static double DistanceToTargetX()  
    {
        NetworkTableEntry tx = table.getEntry("tx");
        double targetOffsetAngle_Horizontal = tx.getDouble(0.0);


        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = 40.0; 
        // distance from the target to the floor
        double goalHeightCM = 112.5; 
        
        double angleToGoalDegrees = targetOffsetAngle_Horizontal;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        SmartDashboard.putNumber("tx", Math.tan(angleToGoalRadians));

        //calculate distance
        return DistanceToTarget() * Math.tan(angleToGoalRadians);
    }

}