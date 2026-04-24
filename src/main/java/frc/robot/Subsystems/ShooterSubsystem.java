package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    InterpolatingDoubleTreeMap intTreeMap = new InterpolatingDoubleTreeMap();
    private static TalonFX shooterKrakenDir = new TalonFX(31, "ShooterCAN");
    private static TalonFX shooterKrakenEsq = new TalonFX(32, "ShooterCAN");
    public static double m_currentSetpoint = 0.0; // Armazena o setpoint de RPM atual
    private final PIDController pid = new PIDController(0.008, 0.0, 0.01);
    // private final PIDController pid = new PIDController(0.1, 0.0, 0.0);

    boolean isShooting;
    double shooterVoltage;
    MegaTagSubsystem mgTag = new MegaTagSubsystem();

    public ShooterSubsystem(){
        shooterKrakenEsq.setControl(new Follower(shooterKrakenDir.getDeviceID(), MotorAlignmentValue.Opposed));
        isShooting = false;
        intTreeMap.put(ShooterConstants.kShooterLowSpeedRPM, 0.0);
        intTreeMap.put(ShooterConstants.kShooterHighSpeedRPM, 4.0);
        // rpmMap.put(300.0, ShooterConstants.kShooterHighSpeedRPM);
    }


    public void applyVelocity(double SpeedRPM, double Kg){
        if(SpeedRPM != 0){
            m_currentSetpoint = SpeedRPM; // Atualiza o setpoint armazenado
            shooterVoltage += pid.calculate(shooterKrakenDir.getVelocity().getValueAsDouble() ,SpeedRPM / 60);
            shooterKrakenDir.setVoltage(shooterVoltage);
            SmartDashboard.putNumber("Shooter SetPoint", m_currentSetpoint);
        }else{
            m_currentSetpoint = SpeedRPM;
            shooterKrakenDir.setVoltage(0);
            SmartDashboard.putNumber("Shooter SetPoint", m_currentSetpoint);
        }
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
            SmartDashboard.putNumber("Shooter Speed", getShooterVelocityRPM());
            if(isShooting == false){
                applyVelocity(0, 0);
            }else{
                // applyVelocity(LimeLightSubsystem.calculateSpeed() * ShooterConstants.kCylinderMultiplierRPM, 0);
                // applyVelocity( 2900, 0); //testRoutine
                applyVelocity(MathUtil.clamp((142.857 * MegaTagSubsystem.getDistanceToHub()) + 2671.429, 0, 3500), 0);

        }
        }else{
            isShooting = false;
        }
    }

}
