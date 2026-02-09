package frc.robot.Commands;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {

    private SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private Supplier<Boolean> fieldOrientedFunction;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private Supplier<Boolean> rotationButton;

    private PS5Controller controller = new PS5Controller(0);

    public SwerveCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> rotationButton) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.rotationButton = rotationButton;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed;
        double ySpeed;
        double turningSpeed ;
        if (controller.getPSButton()){

            xSpeed = 0;
            ySpeed = -0.1;
            turningSpeed = turningSpdFunction.get();
        }
        else if (rotationButton.get()){
            double RotNow = swerveSubsystem.getHeading();
            xSpeed = 0;
            ySpeed = 0;
            while(swerveSubsystem.getHeading() <= RotNow +90 + 3 && swerveSubsystem.getHeading() >= RotNow +90 - 3){
                PIDController pid = new PIDController(0.01,0,0);
                turningSpeed = MathUtil.clamp(pid.calculate(-swerveSubsystem.getHeading(), RotNow +90), -0.3, 0.3);
                SmartDashboard.putNumber("Robot Heading", swerveSubsystem.getHeading());
                SmartDashboard.putNumber("Turning setpoint", RotNow + 90);

                OutputToWheels(xSpeed, ySpeed, turningSpeed);
            }
            turningSpeed=0;
            
        }
        else{
            xSpeed = xSpdFunction.get();
            ySpeed = ySpdFunction.get();
            turningSpeed = turningSpdFunction.get();
        }

        OutputToWheels(xSpeed, ySpeed, turningSpeed);
    }

    public void OutputToWheels(double xspd, double yspd, double turningspd){
                // 2. Apply deadband
        xspd = Math.abs(xspd) > OIConstants.kDeadband ? xspd : 0.0;
        yspd = Math.abs(yspd) > OIConstants.kDeadband ? yspd : 0.0;
        turningspd = Math.abs(turningspd) > OIConstants.kDeadband ? turningspd : 0.0;

        // 3. Make the driving smoother
        xspd = xLimiter.calculate(xspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        yspd = yLimiter.calculate(yspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningspd = turningLimiter.calculate(turningspd)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        // if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( xspd, yspd, turningspd, swerveSubsystem.getRotation2d());
        // } else {
        //     // Relative to robot
        //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, -ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
       
        // }

        if (fieldOrientedFunction.get()){
            swerveSubsystem.zeroHeading();
        }

        

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
        return false;
    }
}