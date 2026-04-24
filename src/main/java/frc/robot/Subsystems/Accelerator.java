package frc.robot.Subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Accelerator extends SubsystemBase{
    private static TalonFX acceleratorKraken = new TalonFX(33, "ShooterCAN");
    private final PIDController pid = new PIDController(0.05, 0, 0);
    double velocityOutput = 0;
    Boolean isShooting = false;
    public static double accelSetPoint;

    public Accelerator(){
        velocityOutput = 0;
        isShooting = false;
    }

    
    public void applyVelocity(double SpeedRPM, double Kg){
        if(SpeedRPM != 0){
            accelSetPoint = SpeedRPM;
            velocityOutput += pid.calculate(acceleratorKraken.getVelocity().getValueAsDouble(), -SpeedRPM / 60 * (24/18));
            acceleratorKraken.setVoltage(velocityOutput);
            SmartDashboard.putNumber("Accelerator SetPoint", SpeedRPM * (24/18));
        }else{
            accelSetPoint = SpeedRPM;
            acceleratorKraken.setVoltage(0);
            SmartDashboard.putNumber("Accelerator SetPoint", 0);
        }
    }

    public void applyVelocityMS(double MS, double Kg){
        velocityOutput += pid.calculate(acceleratorKraken.getVelocity().getValueAsDouble(), -MS/ 0.00765 / 60 * (24/18));
        acceleratorKraken.setVoltage(velocityOutput);
        SmartDashboard.putNumber("Accelerator SetPointMS", MS/ 0.00765 * (24/18));
    }
    public static double getAcceleratorVelocityRPM(){
        return acceleratorKraken.getVelocity().getValueAsDouble() * 60;
    }
    public static double getAcceleratorVelocityMS(){
        return  0.00765 * acceleratorKraken.getVelocity().getValueAsDouble() * 60;
    }

    public void DebugAccel(){
        SmartDashboard.putNumber("Accelerator Speed", -acceleratorKraken.getVelocity().getValueAsDouble() * 60);
        // SmartDashboard.putNumber("accelBallVelocityMS", 0.00765 * getAcceleratorVelocityRPM());
    }
    public void SetAccelerator(){
        isShooting = !isShooting;
    }

    public void TurnOffAccelerator(){
        isShooting = false;
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()){
            // DebugAccel();
            if(isShooting == false){
                applyVelocity(0, 0);
            }else if (isShooting == true){
                // if(LimeLightSubsystem.DistanceToTarget() <= 290 ){
                //     // applyVelocity(LimeLightSubsystem.calculateSpeed() * ShooterConstants.kAcceleratorMultiplierRPM, 0);
                //     applyVelocity(3210,0 );
                // }else{
                    applyVelocity(2910,0 );
                // }
            }
        }else{
            velocityOutput = 0;
            isShooting = false;
        }
    }
}
