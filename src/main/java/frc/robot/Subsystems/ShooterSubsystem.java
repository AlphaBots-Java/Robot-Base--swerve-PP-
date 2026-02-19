package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private static TalonFX shooterKraken1 = new TalonFX(0, "canBUS");
    private static TalonFX shooterKraken2 = new TalonFX(1, "canBUS");

    TalonFXConfiguration configs = new TalonFXConfiguration();
    public ShooterSubsystem(){
        shooterKraken2.setControl(new Follower(shooterKraken1.getDeviceID(), MotorAlignmentValue.Opposed));


        configs.Feedback.SensorToMechanismRatio = 1.0;

        configs.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
        configs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        configs.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
        configs.Slot0.kI = 0; // no output for integrated error
        configs.Slot0.kD = 0; // no output for error derivative

        shooterKraken1.getConfigurator().apply(configs);
    }

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);


    public void applyVelocity(double SpeedRPS, double Kg){
        shooterKraken1.setControl(m_request.withVelocity(SpeedRPS).withFeedForward(Kg));
    }

    public static double getShooterVelocityRPM(){
        return shooterKraken1.getVelocity().getValueAsDouble() * 60;
    }

}
