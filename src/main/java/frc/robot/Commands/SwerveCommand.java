package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private final PS5Controller controller = new PS5Controller(0);

    private final PIDController xPID = new PIDController(2.5, 0.0, 0.2);
    private final PIDController yPID = new PIDController(2.5, 0.0, 0.2);
    private final PIDController omegaPID = new PIDController(2.0, 0.0, 0.2);

    public SwerveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    private double clamp(double value, double max) {
        if (value > max) return max;
        if (value < -max) return -max;
        return value;
    }

    @Override
    public void execute() {

        double xSpeed;
        double ySpeed;
        double turningSpeed;

        boolean joystickActive =
                Math.abs(xSpdFunction.get()) > OIConstants.kDeadband ||
                Math.abs(ySpdFunction.get()) > OIConstants.kDeadband ||
                Math.abs(turningSpdFunction.get()) > OIConstants.kDeadband;

        if (controller.getPSButton()) {

            xSpeed = 0;
            ySpeed = -0.1;
            turningSpeed = turningSpdFunction.get();

        }
        else if (!joystickActive) {

            ChassisSpeeds current =
                    DriveConstants.kDriveKinematics.toChassisSpeeds(
                            swerveSubsystem.getModuleStates()
                    );

            xSpeed = xPID.calculate(current.vxMetersPerSecond, 0.0);
            ySpeed = yPID.calculate(current.vyMetersPerSecond, 0.0);
            turningSpeed = omegaPID.calculate(current.omegaRadiansPerSecond, 0.0);

            xSpeed = clamp(xSpeed, 2.5);
            ySpeed = clamp(ySpeed, 2.5);
            turningSpeed = clamp(turningSpeed, 2.0);

        }
        else {

            xSpeed = xSpdFunction.get();
            ySpeed = ySpdFunction.get();
            turningSpeed = -turningSpdFunction.get();
        }

        outputToWheels(xSpeed, ySpeed, turningSpeed);
    }

    public void outputToWheels(double xspd, double yspd, double turningspd) {

        // deadband
        xspd = Math.abs(xspd) > OIConstants.kDeadband ? xspd : 0.0;
        yspd = Math.abs(yspd) > OIConstants.kDeadband ? yspd : 0.0;
        turningspd = Math.abs(turningspd) > OIConstants.kDeadband ? turningspd : 0.0;

        // smoothing (igual teu sistema)
        xspd = xLimiter.calculate(xspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        yspd = yLimiter.calculate(yspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningspd = turningLimiter.calculate(turningspd)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xspd,
                        yspd,
                        turningspd,
                        swerveSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

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