package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Positive Out extender
// positive shoots axis out the robot

public class CatcherSubsystem extends SubsystemBase{
    private final TalonFX krakenExtensor = new TalonFX(51, "BallSystemCAN");
    private final TalonFX krakenCatcher = new TalonFX(52, "BallSystemCAN");
    private final CANcoder catcherCANcoder = new CANcoder(50, "BallSystemCAN");
    boolean isExtended = false;
    int isCatching = 0;
    double newSpeed = 0;
    double clampedSetPoint = 0;
    double axisSpeedSetpoint = 0;

    private PIDController catcherController = new PIDController(2.3, 0.0, 0.1);
    // private PIDController catcherController = new PIDController(0.5, 0, 0); Debug PID values
    private PIDController AxisController = new PIDController(0.03, 0, 0);

    public CatcherSubsystem(){
        isCatching = 0;
        isExtended = false;
        newSpeed = 0;
        clampedSetPoint = 0;
        axisSpeedSetpoint = 0;
    }


    public void setCatcherExtenderMM(double setPoint){
        clampedSetPoint = setPoint;
        krakenExtensor.setVoltage(catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint)); 
        // if(isCatching == 2 && catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint) < 0.5){
        //     isCatching = 0;
        // }
    }

    // public void setCatcherExtenderCREU(double setPoint){
    //     new Thread(() -> {
    //         clampedSetPoint = setPoint;
    //         while(!(catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint) < 0.5 && catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint) > -0.5)){
    //             krakenExtensor.setVoltage(catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint)); 
    //         }
    //         krakenExtensor.setVoltage(0);
    //         return;
    //     }).start(); 
    // }


    public void SetExtendedCREU(){
        // if(isExtended == false){
        //     setCatcherExtenderCREU(2.6);
        // }
        // if(isExtended == true){
        //     setCatcherExtenderCREU(0);
        // }
        isExtended = !isExtended;
    }

    public void SetExtended(){
        // if(isExtended == false){
        //     setCatcherExtenderMM(2.6);
        //     // setExpellingTrue();
        // }
        // if(isExtended == true){
        //     setCatcherExtenderMM(0);
        // }
        isExtended = !isExtended;
    }



    public void SetExtendedTrue(){
        // setCatcherExtenderMM(2.6);
        isExtended = true;
    }

    public void SetCatching(){
        if(isCatching == 0 || isCatching == 2){
            isCatching = 1;
        }
        else{
            isCatching = 0;
        }
    }
    public void SetCatchingTrue(){
        isCatching = 1;
    }

    public void SetCatchingFalse(){
        isCatching = 0;
    }

    public void setExpellingTrue(){
        isCatching = 2;
    }

    public void setDropping(){
        if(isCatching == 0 || isCatching == 1){
            isCatching = 2;
        }
        else{
            isCatching = 0;
        }
    }


    void DebugCatcher(){
        SmartDashboard.putNumber("Catcher CanCoder", catcherCANcoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("extenderSetpoint", clampedSetPoint);

    }

    @Override
    public void periodic(){
        DebugCatcher();
        if(DriverStation.isEnabled()){
            if(isExtended == false){
                setCatcherExtenderMM(0);
            }else if (isExtended == true){
                setCatcherExtenderMM(3.0);
            }

            if(isCatching == 1){
                // setCatcherVelocity(CatcherSpeed);
                krakenCatcher.set(-0.9);
            }else if(isCatching == 2){
                // setCatcherVelocity(-CatcherSpeed);
                krakenCatcher.set(0.9);
            }
            else if(isCatching == 0){
                // setCatcherVelocity(0.0);
                krakenCatcher.set(0.0);
            }
        }else{
            isCatching = 0;
            krakenCatcher.set(0);
            // setCatcherVelocity(0.0);
        }
        DebugCatcher();
        
    }
}
