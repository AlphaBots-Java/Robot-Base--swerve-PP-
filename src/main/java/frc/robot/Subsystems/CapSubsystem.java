package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.TreeMap;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.LimeLightCommand;
import frc.robot.Constants.ShooterConstants;

public class CapSubsystem extends SubsystemBase{
    public TalonFX AnglerMotor = new TalonFX(30, "ShooterCAN");
    CANcoder capThroghBourne = new CANcoder(2, "ShooterCAN");
    InterpolatingDoubleTreeMap intTreeMap = new InterpolatingDoubleTreeMap();
    // 440 - 15
    // 160 - 
    


    PIDController capPID = new PIDController(0.1 ,0.0, 0.0);
    boolean isExtended = false;

    public CapSubsystem(){
        // capPID.enableContinuousInput(-180, 180);
        isExtended = false;
        intTreeMap.put(290.0, 16.0);
        intTreeMap.put(260.0, 9.0);
        intTreeMap.put(230.0, 7.0);
        intTreeMap.put(210.0, 5.0);
        intTreeMap.put(180.0, 3.0);

    }

    boolean CanShoot(){
        return ShooterSubsystem.getShooterVelocityMS() > Accelerator.getAcceleratorVelocityMS(); //colocar em ms
               
    }

    public double getEncoderValue(){
        double angle = capThroghBourne.getPosition().getValueAsDouble() * 360;
        return angle;
    }
    public double getStepDegrees(){
        return (getEncoderValue() * ShooterConstants.kCapStep); //O angulo real do cap do shootador
    }
    public void setAngleDegrees(double angle){
        MathUtil.clamp(angle, 0.0, 21.0);
        AnglerMotor.setVoltage(capPID.calculate(getEncoderValue() , -angle * 14.87));
        SmartDashboard.putNumber("CapPosition", getEncoderValue() / 14.87);
        SmartDashboard.putNumber("AnglerSetPoint", angle * 14.87);
        SmartDashboard.putNumber("voltageOut", (angle/ShooterConstants.kCapStep)/180 * Math.PI);
    }

    public void RunExtender(){
        isExtended = !isExtended;
    }

    public boolean isExtended() {
        return isExtended;
    }

    public double getAngleForDistance(double distance) {
        return intTreeMap.get(distance);
    }

    @Override
     public void periodic(){
        if(DriverStation.isEnabled()){
            if(isExtended == false){
                setAngleDegrees(0);
            }else if (isExtended == true){
                // if(LimeLightSubsystem.DistanceToTarget() <= 290){
                //     setAngleDegrees(intTreeMap.get(LimeLightSubsystem.DistanceToTarget()));
                // }else{
                    setAngleDegrees(intTreeMap.get(LimeLightSubsystem.DistanceToTarget()));
                // }
                
                // intTreeMap.get(LimeLightSubsystem.DistanceToTarget())
            }
        }else{
            isExtended = false;
            setAngleDegrees(0);
        }
     }  
}
