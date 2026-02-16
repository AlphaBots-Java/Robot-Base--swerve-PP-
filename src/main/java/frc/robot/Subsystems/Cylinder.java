package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cylinder extends SubsystemBase {
    private static TalonFX cylinderKraken = new TalonFX(0, "canBUS");

    private final TalonFXConfiguration accelConfigs = new TalonFXConfiguration();

    private double AcceleratorSpeedMargin = 2;
    private double ShooterSpeedMargin = 2;

    public Cylinder(){
        accelConfigs.Feedback.SensorToMechanismRatio = 9;

        accelConfigs.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
        accelConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        accelConfigs.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
        accelConfigs.Slot0.kI = 0; // no output for integrated error
        accelConfigs.Slot0.kD = 0; // no output for error derivative
        // 0.01534*rpm Velocidade da bola 
        // 0.00266*rpm velocidade do acelerador m/s


        cylinderKraken.getConfigurator().apply(accelConfigs);   
    }
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    
    public void applyVelocity(double SpeedRPS, double Kg){

        double ballVelocity = 0.01534 * getCylinderVelocityRPM();
        double accelVelocity = 0.00266 * Accelerator.getAcceleratorVelocityRPM();
        double shooterVelocity = 0.00266 * ShooterSubsystem.getShooterVelocityRPM();

        boolean canShoot = ballVelocity > accelVelocity - AcceleratorSpeedMargin && ballVelocity > shooterVelocity - ShooterSpeedMargin;

        if(canShoot){
            cylinderKraken.setControl(m_request.withVelocity(SpeedRPS).withFeedForward(Kg));
        }else{
            cylinderKraken.set(0);
        }
        SmartDashboard.putBoolean("Posso Chutar?", canShoot);
        
    }

    public double getCylinderVelocityRPM(){
        return cylinderKraken.getVelocity().getValueAsDouble() * 60;
    }


}
