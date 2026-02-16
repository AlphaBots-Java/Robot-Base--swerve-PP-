package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class CatcherSubsystem extends SubsystemBase{
    private final TalonFX krakenAngler = new TalonFX(0, "canBUS");
    private final TalonFX krakenCatcher = new TalonFX(1, "canBUS");
    private final CANcoder catcherCANcoder = new CANcoder(2);

    private PIDController catcherController = new PIDController(0, 0, 0);

    public void setCatcherExtenderMM(double setPoint){
        krakenAngler.setVoltage(catcherController.calculate(catcherCANcoder.getPosition().getValueAsDouble() * ShooterConstants.kCatcherRotationsToMM ,setPoint));
    }

    public void setCatcherVelocity(Double CatcherSpeed){
        krakenCatcher.set(CatcherSpeed);
    }
}
