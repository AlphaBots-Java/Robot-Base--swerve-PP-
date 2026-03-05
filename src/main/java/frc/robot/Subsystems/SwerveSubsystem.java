package frc.robot.Subsystems; 
import java.util.Stack;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    SwerveModulePosition[] saas;
        
    private final Pigeon2 pigeon = new Pigeon2(5, "BallSystemCAN");
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(0),
                                                                        new SwerveModulePosition[]{
                                                                            frontLeft.GetModulePosition(),
                                                                            frontRight.GetModulePosition(), 
                                                                            backLeft.GetModulePosition(), 
                                                                            backRight.GetModulePosition(),}
                                                                            );
    
    Stack<Double> lastXvalues = new Stack<>();
    int maxXvalues = 5;
    public static double xStandardDeviation = 0;
    Stack<Double> lastYvalues = new Stack<>();
    int maxYvalues = 5;
    public static double yStandardDeviation = 0;

    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        double xStandardDeviation = 0;
        double yStandardDeviation = 0;

    RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner RobotConfig!", e.getStackTrace());
            throw new RuntimeException("PathPlanner config failed to load", e);
        }
        
    
        AutoBuilder.configure(this::getPose,
                              this::resetOdometry,
                              () -> DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                                                                              frontRight.getState(),
                                                                              backLeft.getState(), 
                                                                              backRight.getState()), 
                              (speeds, feedforward) -> driveRobotRelative(speeds),  
                              new PPHolonomicDriveController( // PathPlanner`s lib for controlling drivetrains
                              new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), 
                              new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.15)),
                              config ,
                              () -> {
                                return false;
                              },
                              this);
        // } catch (Exception e) {
        //     e.printStackTrace();
        // }
    }

    public void zeroHeading() {
        pigeon.reset();
    }

    //Correcao da orientacao do robo (-90)
    public double getHeading() {
        return pigeon.getRotation2d().getDegrees();
        // return -90;
    }

    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }


    public Pose2d getPose() {
       return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
       odometer.resetPosition(getRotation2d(), new SwerveModulePosition[]{
                                                frontLeft.GetModulePosition(),
                                                frontRight.GetModulePosition(), 
                                                backLeft.GetModulePosition(), 
                                                backRight.GetModulePosition()}, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[]{frontLeft.GetModulePosition(),
                                                                    frontRight.GetModulePosition(),
                                                                    backLeft.GetModulePosition(), 
                                                                    backRight.GetModulePosition()});


        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("angle0", frontLeft.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("angle1", backLeft.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("angle2", frontRight.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("angle3", backRight.getAbsoluteEncoderDeg());

        SmartDashboard.putNumber("velocity0", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("velocity1", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("velocity2", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("velocity3", backRight.getDriveVelocity());

        SmartDashboard.putNumber("Limelight-Distance-toTarget", LimeLightSubsystem.DistanceToTarget());

        // Calculate the derivative of robot`s position according to the apriltag

        lastXvalues.push(LimeLightSubsystem.DistanceToTargetX());
        if(lastXvalues.size() > maxXvalues){
            lastXvalues.pop();
        }
        xStandardDeviation = Math.sqrt((GetStackSum(lastXvalues) - GetStackMean(lastXvalues))/lastXvalues.size() - 1);

        lastYvalues.push(LimeLightSubsystem.DistanceToTarget());
        if(lastYvalues.size() > maxYvalues){
            lastYvalues.pop();
        }
        yStandardDeviation = Math.sqrt((GetStackSum(lastYvalues) - GetStackMean(lastYvalues))/lastYvalues.size() - 1);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], frontLeft.getRotation2d());
        frontRight.setDesiredState(desiredStates[1], frontRight.getRotation2d());
        backLeft.setDesiredState(desiredStates[2], backLeft.getRotation2d());
        backRight.setDesiredState(desiredStates[3], backRight.getRotation2d());
    }

    public void driveRobotRelative(ChassisSpeeds Mspeeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(Mspeeds);
        this.setModuleStates(moduleStates);
    }

    
    public void playInstruments(){
        frontLeft.orchestra.play();
        frontRight.orchestra.play();
        backLeft.orchestra.play();
        backRight.orchestra.play();
    }
    Double GetStackSum(Stack<Double> stack){
        double sum = 0;
        for (double value : stack) {
            sum += value;
        }
        return sum;
    }
    Double GetStackMean(Stack<Double> stack){
        return GetStackSum(stack) / stack.size();
    }
    
}