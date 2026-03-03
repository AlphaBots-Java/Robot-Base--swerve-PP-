package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private static TalonFX shooterKrakenDir = new TalonFX(31, "ShooterCAN");
    private static TalonFX shooterKrakenEsq = new TalonFX(32, "ShooterCAN");
    PIDController pid = new PIDController(0.01, 0, 0);
    double ShooterVoltage = 0;
    static Double ShooterSetPoint;
    boolean isShooting = false;

    public ShooterSubsystem(){
        shooterKrakenEsq.setControl(new Follower(shooterKrakenDir.getDeviceID(), MotorAlignmentValue.Opposed));
        isShooting = false;
        ShooterVoltage = 0;
        ShooterSetPoint = 0.0;
    }


    public void applyVelocity(double SpeedRPM, double Kg){
        ShooterVoltage += pid.calculate(shooterKrakenDir.getVelocity().getValueAsDouble() ,SpeedRPM / 60);
        shooterKrakenDir.setVoltage(ShooterVoltage);
        SmartDashboard.putNumber("Shooter SetPoint", SpeedRPM);
    }

    public static double getShooterVelocityRPM(){
        return shooterKrakenDir.getVelocity().getValueAsDouble() * 60;
    }

    public void DebugShooter(){
        // SmartDashboard.putNumber("Shooter Speed", shooterKrakenDir.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("ShooterBallVelocityMS", 0.01193 * getShooterVelocityRPM());
    }

    
    public void ActOrNotShooter(){
       isShooting = !isShooting;
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()){
            ShooterSetPoint = LimeLightSubsystem.calculateSpeed();
            if(isShooting == false){
                applyVelocity(0, 0);
            }else if (isShooting == true){
                applyVelocity(2900, 0);
            }
        }
        else{
            isShooting = false;
            ShooterVoltage = 0;
            ShooterSetPoint = 0.0;
        }
    }

}
