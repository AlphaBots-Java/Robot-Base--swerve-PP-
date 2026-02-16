package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{
    
    ShooterSubsystem shooter = new ShooterSubsystem();
    Supplier<Boolean> OnButton;
    static Double ShooterSpeed;
    Boolean isShooting;

    public ShooterCommand(ShooterSubsystem shooter ,Supplier<Boolean> OnButton){
        this.shooter = shooter;
        this.OnButton = OnButton;
    }

    @Override
    public void initialize() {
        isShooting = false;
    }

    @Override
    public void execute(){
        calculateSpeed();

        if(OnButton.get() && isShooting == false){
            shooter.applyVelocity(ShooterSpeed, 0);
            isShooting = true;
        }else if (OnButton.get() && isShooting == true){
            shooter.applyVelocity(0, 0);
            isShooting = false;
        }
    }
    
    void calculateSpeed(){
        if (LimeLightSubsystem.DistanceToTarget() < ShooterConstants.kDistanceForHighSpeedMeters){
            ShooterSpeed = ShooterConstants.kShooterLowSpeedRPM;
        }else{
            ShooterSpeed = ShooterConstants.kShooterHighSpeedRPM;
        }
    }

    public static double getShooterSpeed(){
        return ShooterSpeed;
    }
}
