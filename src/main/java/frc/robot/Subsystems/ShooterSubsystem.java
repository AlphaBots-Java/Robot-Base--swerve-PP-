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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterSubsystem extends SubsystemBase{
    private static TalonFX shooterKrakenDir = new TalonFX(31, "ShooterCAN");
    private static TalonFX shooterKrakenEsq = new TalonFX(32, "ShooterCAN");
    public static double m_currentSetpoint = 0.0; // Armazena o setpoint de RPM atual
    private final PIDController pid = new PIDController(0.02, 0, 0.0001);
    double shooterVoltage;

    public ShooterSubsystem(){
        shooterKrakenEsq.setControl(new Follower(shooterKrakenDir.getDeviceID(), MotorAlignmentValue.Opposed));

        // rpmMap.put(300.0, ShooterConstants.kShooterHighSpeedRPM);
    }


    public void applyVelocity(double SpeedRPM, double Kg){
        m_currentSetpoint = SpeedRPM; // Atualiza o setpoint armazenado
        shooterVoltage += pid.calculate(shooterKrakenDir.getVelocity().getValueAsDouble() ,SpeedRPM / 60);
        shooterKrakenDir.setVoltage(shooterVoltage);
        SmartDashboard.putNumber("Shooter SetPoint", m_currentSetpoint);
    }

    public double getCurrentSetpoint() {
        return m_currentSetpoint;
    }

    public static double getShooterVelocityRPM(){
        return shooterKrakenDir.getVelocity().getValueAsDouble() * 60;
    }
    public static double getShooterVelocityMS(){
        return 0.01193 * getShooterVelocityRPM();
    }

    public void DebugShooter(){
        SmartDashboard.putNumber("Shooter Speed", getShooterVelocityRPM());
        SmartDashboard.putNumber("ShooterBallVelocityMS", 0.01193 * getShooterVelocityRPM());
    }


    @Override
    public void periodic() {
        // A lógica de controle foi movida para o ShooterCommand.
        // O método periodic() deve ser usado para coisas que precisam rodar
        // constantemente, como atualizar a SmartDashboard.
        DebugShooter();
    }

}
