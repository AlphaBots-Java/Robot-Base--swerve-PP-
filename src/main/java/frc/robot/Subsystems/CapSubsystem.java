package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class CapSubsystem extends SubsystemBase{
    public TalonFX AnglerMotor = new TalonFX(30, "ShooterCAN");
    CANcoder capThroghBourne = new CANcoder(2, "ShooterCAN");
    InterpolatingDoubleTreeMap intTreeMap = new InterpolatingDoubleTreeMap();
    // 440 - 15
    // 160 - 
    
    
    
    PIDController capPID = new PIDController(0.1 ,0, 0.01);
    boolean autoAlign = true;
    boolean isExtended = false;

    public CapSubsystem(){
        // capPID.enableContinuousInput(-180, 180);
        isExtended = false;
        autoAlign = true;
        // intTreeMap.put(290.0, 16.0);
        // intTreeMap.put(260.0, 9.0);
        // intTreeMap.put(230.0, 7.0);
        // intTreeMap.put(210.0, 5.0);
        // intTreeMap.put(180.0, 3.0);

        intTreeMap.put(161.0, 3.5);
        intTreeMap.put(214.0, 6.1);
        intTreeMap.put(245.0, 8.1);
        intTreeMap.put(275.0, 10.2);
        intTreeMap.put(300.0, 12.5);
        intTreeMap.put(350.0, 16.1);
        intTreeMap.put(380.0, 18.1);

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

        double distanceRaw =  MegaTagSubsystem.getDistanceToHub();
        double targetAngle = intTreeMap.get(distanceRaw * 100);
        MathUtil.clamp(angle, 0.0, 21.0);
        AnglerMotor.setVoltage(capPID.calculate(getEncoderValue() , -angle * 14.87));
        SmartDashboard.putNumber("CapPosition", getEncoderValue() / 14.87);
        SmartDashboard.putNumber("Cap ABS", capThroghBourne.getPosition().getValueAsDouble() * 360 / 14.87);
        SmartDashboard.putNumber("AnglerSetPoint", angle * 14.87);
        SmartDashboard.putNumber("voltageOut", (angle/ShooterConstants.kCapStep)/180 * Math.PI);
        SmartDashboard.putNumber("Distance Raw", distanceRaw);
        SmartDashboard.putNumber("Distance map", distanceRaw * 100);
        SmartDashboard.putNumber("Target Angle Map", targetAngle);
    }

    public void RunExtender(){
        isExtended = !isExtended;
    }

    public void ChangeAlignMode(){
        autoAlign = !autoAlign;
    }

    public boolean isExtended() {
        return isExtended;
    }

    public double getAngleForDistance(double distance) {
        return intTreeMap.get(distance);
    }

    @Override
     public void periodic(){
        if (isExtended){

                double distance = MegaTagSubsystem.getDistanceToHub() * 100;
                double targetAngle = intTreeMap.get(distance);
                double clampedOutput = MathUtil.clamp(targetAngle, 0, 20);

                setAngleDegrees(clampedOutput);
            
            }
        else {
            setAngleDegrees(0);
        }
        // if(DriverStation.isEnabled()){
        //     if(isExtended == false){
        //         setAngleDegrees(0);
        //     }else if (isExtended == true){
        //         // if(autoAlign == true){
        //             double clampedOutput = MathUtil.clamp(intTreeMap.get(MegaTagSubsystem.getDistanceToHub() * 100), 0, 20);
        //             setAngleDegrees(clampedOutput);
        //             // setAngleDegrees(19.6);

        //         // }else{
        //         //     setAngleDegrees(18);
        //         // }
        //     }
        // }else{
        //     isExtended = false;
        //     autoAlign = true;
        //     setAngleDegrees(0);
        // }
     }  
}
