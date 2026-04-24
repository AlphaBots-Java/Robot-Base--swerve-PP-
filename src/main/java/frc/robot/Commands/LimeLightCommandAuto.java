package frc.robot.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class LimeLightCommandAuto extends Command{
    LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public PIDController pid = new PIDController(0.03, 0, 0.0);
    private SlewRateLimiter turningLimiter;
    Timer timer = new Timer();


    public LimeLightCommandAuto(LimeLightSubsystem limelight, SwerveSubsystem swerve){
        limeLightSubsystem = limelight;
        swerveSubsystem = swerve;
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double tx = LimelightHelpers.getTX(LimelightConstants.limelightTableName);
        double aligningMaxSpd = 0.17;
        

        double correctRotation = MathUtil.clamp(pid.calculate(tx, 0), -aligningMaxSpd, aligningMaxSpd);

        SmartDashboard.putNumber("tx", correctRotation);

        OutputToWheels(correctRotation);
    }

    public void OutputToWheels(double turningspd){
        // 2. Apply deadband
        turningspd = Math.abs(turningspd) > AutoConstants.kAngleDeadband ? turningspd : 0.0;
    
        // 3. Make the driving smoother
        turningspd = turningLimiter.calculate(turningspd)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningspd, swerveSubsystem.getRotation2d());

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
        if(timer.hasElapsed(1)){
            return true;
        }
        else{
            return false;
        }
    }

}
