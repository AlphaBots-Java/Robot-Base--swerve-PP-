package frc.robot.Subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Commands.LimeLightCommand;
import frc.robot.Constants.LimelightConstants;

public class MegaTagSubsystem extends SubsystemBase{
    public static double MTX = 0;
    public static double MTY = 0;
    static double lastHubDistance = 0;

    private int[] validIDs;
    SwerveSubsystem swerveSubsystem;
    // private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private boolean doRejectUpdate = false;

    public MegaTagSubsystem(){
        validIDs = new int[]{10, 11, 9, 2, 5, 8, 21, 24, 26, 25, 18, 27};
        LimelightHelpers.SetFiducialIDFiltersOverride(LimelightConstants.limelightTableName, validIDs);
        swerveSubsystem = new SwerveSubsystem();
        doRejectUpdate = false;
    }

    public static double getXDistanceToHub(){
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            
            return LimelightConstants.distanceHubXMeters - MTX;
        }else{
            // return -16.08 + LimelightConstants.distanceHubXMeters + swerve.getPose().getX();
            return MTX - (16.08 - LimelightConstants.distanceHubXMeters);

        }
    }

    public static double getYDistanceToHub(){
        return MTY - LimelightConstants.distanceHubYMeters;
    }

    public static double getDistanceToHub(){
        if(LimelightHelpers.getTV(LimelightConstants.limelightTableName)){
            lastHubDistance = Math.sqrt(Math.pow(getXDistanceToHub(), 2) + Math.pow(getYDistanceToHub(), 2));
            return Math.sqrt(Math.pow(getXDistanceToHub(), 2) + Math.pow(getYDistanceToHub(), 2));
        }
        return lastHubDistance;
        
        
    }

    public void PrintThings(LimelightHelpers.PoseEstimate mt){
        SmartDashboard.putNumber("DistanceHub", lastHubDistance);
        SmartDashboard.putNumber("DistanceHubY", getYDistanceToHub());
         SmartDashboard.putNumber("DistanceHubX", getXDistanceToHub());
    }
    
    @Override 
    public void periodic(){

        LimelightHelpers.SetRobotOrientation(
        LimelightConstants.limelightTableName,
        swerveSubsystem.getRotation2d().getDegrees(),
        0,0,0,0,0
        );

        LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(
            LimelightConstants.limelightTableName
        );
        SmartDashboard.putNumber("tagCount", mt2 == null ? -1 : mt2.tagCount);
        SmartDashboard.putBoolean("MT2 NULL", mt2 == null);
        if(mt2 == null){
            return;
        }
        PrintThings(mt2);

        MTX = mt2.pose.getX();
        MTY = mt2.pose.getY();

    boolean doRejectUpdate = false;

    if(Math.abs(swerveSubsystem.getTurnRate()) > 360){
        doRejectUpdate = true;
    }

    if(mt2.tagCount == 0){
        doRejectUpdate = true;
    }

    if(!doRejectUpdate){
        swerveSubsystem.odometer.setVisionMeasurementStdDevs(
            VecBuilder.fill(.7,.7,9999999)
        );

        swerveSubsystem.odometer.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds
        );
    }
    
    }
}
