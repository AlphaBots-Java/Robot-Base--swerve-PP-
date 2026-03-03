package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Constants.ShooterConstants;

public class Cylinder extends SubsystemBase {
    private TalonFX cylinderKraken = new TalonFX(35, "BallSystemCAN");
    PIDController controller = new PIDController(0.01, 0, 0);
    // private final TalonFXConfiguration CylinderConfigs = new TalonFXConfiguration();
    double velocityOutput = 0;
    boolean isShooting = false;

    private double AcceleratorSpeedMargin = 2;
    private double ShooterSpeedMargin = 2;

    public Cylinder(){
        // CylinderConfigs.Feedback.SensorToMechanismRatio = 1;
        // CylinderConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // // CylinderConfigs.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
        // // CylinderConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // CylinderConfigs.Slot0.kP = 0.4; // An error of 1 rps results in 0.11 V output
        // CylinderConfigs.Slot0.kI = 0.4; // no output for integrated error
        // CylinderConfigs.Slot0.kD = 0.0; // no output for error derivative
        // // 0.01534*rpm Velocidade da bola

        // // 0.00266*rpm velocidade do acelerador m/s


        // cylinderKraken.getConfigurator().apply(CylinderConfigs);  
        velocityOutput = 0;
        isShooting = false;
        
    }
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public boolean CanShoot(){
        double ballVelocity = 0.01534 * getCylinderVelocityRPM();
        double accelVelocity = 0.00266 * AcceleratorSpeedMargin;
        double shooterVelocity = 0.00266 * ShooterSpeedMargin;

        // return       Accelerator.getAcceleratorVelocityRPM() >= AcceleratorCommand.getAcceleratorSetPoint() 
        //         &&   ShooterSubsystem.getShooterVelocityRPM() >= ShooterCommand.getShooterSetPoint()
        //         &&   shooterVelocity > accelVelocity;
        return true;
    }

    public void applyVelocity(double SpeedRPM, double Kg){
        velocityOutput += controller.calculate(cylinderKraken.getVelocity().getValueAsDouble(), MathUtil.clamp(SpeedRPM / 60 * 9, 0, 650));
        cylinderKraken.setVoltage(velocityOutput);
        SmartDashboard.putNumber("Cylinder SetPoint", velocityOutput);
    }

    public void applyVelocityMS(double SpeedMS, double Kg){
        velocityOutput += controller.calculate(cylinderKraken.getVelocity().getValueAsDouble(), SpeedMS/ 0.01534 / 60 * 9);
        cylinderKraken.setVoltage(velocityOutput);
        SmartDashboard.putNumber("Cylinder SetPoint", SpeedMS * 9);
    }

    public double getCylinderVelocityRPM(){
        return cylinderKraken.getVelocity().getValueAsDouble() / 9 * 60;
    }

    public void DebugCylinder(){
        SmartDashboard.putNumber("Cylinder Speed", cylinderKraken.getVelocity().getValueAsDouble() / (9*60));
        // SmartDashboard.putNumber("CylinderBallVelocityMS", 0.01534 * getCylinderVelocityRPM());
    }
    public void SetCylinder(){
        // if(CanShoot()){
            isShooting = !isShooting;
        // }
    }
    @Override
    public void periodic(){
        if(DriverStation.isEnabled()){
            DebugCylinder();
            SmartDashboard.putBoolean("Posso Chutar", isShooting);
            if(isShooting == false){
                applyVelocity(0, 0);
            }else{
                // applyVelocity(LimeLightSubsystem.calculateSpeed() * ShooterConstants.kCylinderMultiplierRPM, 0);
                applyVelocity( 400, 0); //testRoutine
            }
        }else{
            velocityOutput = 0;
            isShooting = false;
        }
    }


}
