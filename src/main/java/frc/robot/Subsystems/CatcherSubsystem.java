package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cscore.AxisCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// Positive Out extender
// positive shoots axis out the robot

public class CatcherSubsystem extends SubsystemBase{
    private final TalonFX krakenExtensor = new TalonFX(51, "BallSystemCAN");
    private final TalonFX krakenCatcher = new TalonFX(52, "BallSystemCAN");
    private final CANcoder catcherCANcoder = new CANcoder(50, "BallSystemCAN");
    boolean isExtended = false;
    double CatcherSpeed = 1000;
    int isCatching = 0;
    double newSpeed = 0;

    private PIDController catcherController = new PIDController(0.02, 0, 0);
    private PIDController AxisController = new PIDController(0.02, 0, 0);

    public CatcherSubsystem(){
        isCatching = 0;
        isExtended = false;
        newSpeed = 0;
    }

    public void setCatcherExtenderMM(double setPoint){
        double clampedSetPoint = MathUtil.clamp(setPoint, 0, 150);
        krakenExtensor.setVoltage(catcherController.calculate(catcherCANcoder.getPosition().getValueAsDouble() * ShooterConstants.kCatcherRotationsToMM, clampedSetPoint));
    }

    public void setCatcherVelocity(Double CatcherSpeed){
        newSpeed += AxisController.calculate(-krakenCatcher.getVelocity().getValueAsDouble(), CatcherSpeed * 3 / 60);
        krakenCatcher.setVoltage(-MathUtil.clamp(newSpeed, -12, 12));
        SmartDashboard.putNumber("CatcherSetPoint", CatcherSpeed * 3 / 60);
        SmartDashboard.putNumber("CatcherSpeed", krakenCatcher.getVelocity().getValueAsDouble());
    }

    public void SetExtended(){
        isExtended = !isExtended;
    }

    public void SetCatching(){
        if(isCatching == 0 || isCatching == 2){
            isCatching = 1;
        }
        else if(isCatching == 1){
            isCatching = 0;
        }
    }

    public void setDropping(){
        if(isCatching == 0 || isCatching == 1){
            isCatching = 2;
        }
        else if(isCatching == 2){
            isCatching = 0;
        }
    }

    public void SetRetracting(){
        if (isExtended == true){
            if(isCatching == 0){
            isExtended = false;
            isCatching = 1;
            setCatcherExtenderMM(0);
            setCatcherVelocity(CatcherSpeed);
            } else if (isCatching == 1) {
            isExtended = false;
            setCatcherExtenderMM(0);  
            }else{
            isExtended = false;
            isCatching = 1;
            setCatcherExtenderMM(0);
            setCatcherVelocity(CatcherSpeed);
            }
        }else{
             if(isCatching == 0){
            isCatching = 1;
            setCatcherVelocity(CatcherSpeed);
            } else if (isCatching == 1) {
            setCatcherExtenderMM(0);  
            }else{
            isCatching = 1;
            setCatcherVelocity(CatcherSpeed);
            }
        }
    }


    void DebugCatcher(){
        SmartDashboard.putNumber("Catcher CanCoder", catcherCANcoder.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic(){
        if(DriverStation.isEnabled()){
        //     if(isExtended == false){
        //         setCatcherExtenderMM(0);
        //     }else if (isExtended == true){
        //         setCatcherExtenderMM(150);
        //     }
        if(isCatching == 1){
            setCatcherVelocity(CatcherSpeed);
        }else if(isCatching == 2){
            setCatcherVelocity(-CatcherSpeed);
        }
        else if(isCatching == 0){
            setCatcherVelocity(0.0);
        }
        }else{
            // isExtended = false;
            // setCatcherExtenderMM(0);
            isCatching = 0;
            setCatcherVelocity(0.0);
        }
        DebugCatcher();
    }
}
