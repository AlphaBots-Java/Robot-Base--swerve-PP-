package frc.robot.Subsystems;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class LimeLightSubsystem {

    public static boolean hasTarget() {
        boolean tv = LimelightHelpers.getTV(LimelightConstants.limelightTableName);
        return tv;
    }
    public static double getTx() {
        double tx = LimelightHelpers.getTX(LimelightConstants.limelightTableName);
        return tx;
    }
    public static double DistanceToTarget()  
    {
        double ty = LimelightHelpers.getTY(LimelightConstants.limelightTableName);
        double targetOffsetAngle_Vertical = ty;

        // how many degrees back is your limelight rotated from perfectly horizontal?
        double limelightMountAngleDegrees = 19.5; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = 400; 
        // distance from the target to the floor
        double goalHeightCM = 112.5;
        
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        SmartDashboard.putNumber("ty", Math.tan(angleToGoalRadians));

        //calculate distance
        return (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians);
    }
    // public static double calculateSpeed(){
        
    //     InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    //     map.put(1.0, ShooterConstants.kShooterLowSpeedRPM);
    //     map.put(2.0, ShooterConstants.kShooterHighSpeedRPM);

    //     return map.get(LimeLightSubsystem.DistanceToTarget());
    // }

}