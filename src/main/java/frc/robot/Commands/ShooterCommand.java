package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.CandleSubsystem;

public class ShooterCommand extends Command{
    
    private final ShooterSubsystem shooter;
    private final Supplier<Boolean> onButton;
    public boolean isShooting;
    private boolean lastButtonState;
    private CandleSubsystem CANdle = new CandleSubsystem();

    public ShooterCommand(ShooterSubsystem shooter, Supplier<Boolean> onButton){
        this.shooter = shooter;
        this.onButton = onButton;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        isShooting = false;
        lastButtonState = false;
        shooter.applyVelocity(0, 0); // Garante que o shooter comece desligado
    }

    @Override
    public void execute(){
        boolean currentButtonState = onButton.get();
        if (currentButtonState && !lastButtonState) {
            isShooting = !isShooting;
        }
        lastButtonState = currentButtonState;

        if(isShooting){
            // if(LimeLightSubsystem.DistanceToTarget() <= 290){
            //     shooter.applyVelocity(2900, 0);
            //     CANdle.setBlinkBlue();
            // }else{
                shooter.applyVelocity(2900, 0);
            // }
        }else{
            shooter.applyVelocity(0, 0);
        }


        SmartDashboard.putBoolean("isShootingShooter", isShooting);
    }
    
    // void calculateAndSetSpeed(){
    //     double distance = LimeLightSubsystem.DistanceToTarget();

    //     if (distance > 300.0) {
    //         // shooterSetPoint = shooter.getRpmForDistance(distance);
    //         shooterSetPoint = 2900.0;

    //     } else {
    //         shooterSetPoint = 2900.0;
    //     }
    // }
    
    @Override
    public void end(boolean interrupted) {
        shooter.applyVelocity(0, 0);
    }
}