package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class LimeLightCommand extends Command{
    LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public PIDController pid = new PIDController(0.03, 0, 0.0);
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private Supplier<Double> xSpdFunction, ySpdFunction;
    private Supplier<Boolean> endAimSup;
    boolean canGoBack;


    

    public LimeLightCommand(Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,Supplier<Boolean> endAim, LimeLightSubsystem limelight, SwerveSubsystem swerve){
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.endAimSup = endAim;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        limeLightSubsystem = limelight;
        swerveSubsystem = swerve;
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        canGoBack = false;
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        NetworkTableEntry tx = LimeLightSubsystem.table.getEntry("tx");
        double aligningMaxSpd = 0.17;
        

        double correctRotation = MathUtil.clamp(pid.calculate(tx.getDouble(0.0), 0), -aligningMaxSpd, aligningMaxSpd);

        SmartDashboard.putNumber("tx", correctRotation);


        double xSpeed;
        double ySpeed;
        xSpeed = xSpdFunction.get();
        ySpeed = ySpdFunction.get();


        OutputToWheels(xSpeed, ySpeed, correctRotation);
    }

    public void OutputToWheels(double xspd, double yspd,double turningspd){
        // 2. Apply deadband
        turningspd = Math.abs(turningspd) > AutoConstants.kAngleDeadband ? turningspd : 0.0;
        xspd = Math.abs(xspd) > OIConstants.kDeadband ? xspd : 0.0;
        yspd = Math.abs(yspd) > OIConstants.kDeadband ? yspd : 0.0;

        // 3. Make the driving smoother
        xspd = xLimiter.calculate(xspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        yspd = yLimiter.calculate(yspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningspd = turningLimiter.calculate(turningspd)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( xspd, yspd, turningspd, swerveSubsystem.getRotation2d());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        new Thread(() -> {
            try {
                Thread.sleep(100);
                canGoBack = true;
            } catch (Exception e) {
            }
        }).start();
        if(endAimSup.get() == true && canGoBack){
            return true;
        }else{
            return false;
        }
    }

}
