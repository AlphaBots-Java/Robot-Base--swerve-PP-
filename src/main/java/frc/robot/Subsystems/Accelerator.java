package frc.robot.Subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Accelerator extends SubsystemBase{
    private static TalonFX acceleratorKraken = new TalonFX(0, "canBUS");
    private final TalonFXConfiguration accelConfigs = new TalonFXConfiguration();

    public Accelerator(){
        accelConfigs.Feedback.SensorToMechanismRatio = 3/4;

        accelConfigs.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
        accelConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        accelConfigs.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
        accelConfigs.Slot0.kI = 0; // no output for integrated error
        accelConfigs.Slot0.kD = 0; // no output for error derivative


        acceleratorKraken.getConfigurator().apply(accelConfigs);   
    }

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    
    public void applyVelocity(double SpeedRPS, double Kg){
        acceleratorKraken.setControl(m_request.withVelocity(SpeedRPS).withFeedForward(Kg));
    }

    public static double getAcceleratorVelocityRPM(){
        return acceleratorKraken.getVelocity().getValueAsDouble() * 60;
    }
}
