package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private static TalonFX shooterKrakenDir = new TalonFX(31, "ShooterCAN");
    private static TalonFX shooterKrakenEsq = new TalonFX(32, "ShooterCAN");
    public static double m_currentSetpoint = 0.0; // Armazena o setpoint de RPM atual
    private final PIDController pid = new PIDController(0.02, 0, 0.0001);
    boolean isShooting;
    double shooterVoltage;

    public ShooterSubsystem(){
        shooterKrakenEsq.setControl(new Follower(shooterKrakenDir.getDeviceID(), MotorAlignmentValue.Opposed));
        isShooting = false;

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

    public void SetShooter(){
        isShooting = !isShooting;
    }

    public void TurnOffShooter(){
        isShooting = false;
    }

    @Override
    public void periodic() {
        // A lógica de controle foi movida para o ShooterCommand.
        // O método periodic() deve ser usado para coisas que precisam rodar
        // constantemente, como atualizar a SmartDashboard.
        DebugShooter();
        if(DriverStation.isEnabled()){
            if(isShooting == false){
                applyVelocity(0, 0);
            }else{
                // applyVelocity(LimeLightSubsystem.calculateSpeed() * ShooterConstants.kCylinderMultiplierRPM, 0);
                applyVelocity( 2900, 0); //testRoutine
        }
        }else{
            isShooting = false;
        }
    }

}
