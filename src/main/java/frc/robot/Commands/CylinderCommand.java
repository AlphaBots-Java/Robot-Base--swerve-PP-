package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.Subsystems.CandleSubsystem;
import frc.robot.Subsystems.Cylinder;

public class CylinderCommand extends Command{
    
    Cylinder maquinaLavar;
    // CandleSubsystem candle = new CandleSubsystem();
    Supplier<Boolean> OnButton;
    Boolean isShooting;

    public CylinderCommand(Cylinder cylinder, Supplier<Boolean> OnButton){
        maquinaLavar = cylinder;
        this.OnButton = OnButton;
        addRequirements(cylinder);
    }

    @Override
    public void initialize() {
        isShooting = false;
        SmartDashboard.putString("initializedcommand", "FRFR");
    }

    @Override
    public void execute(){
        if (OnButton.get() && isShooting == false){
            StartShooting();
        }
        if (OnButton.get() && isShooting == true){
            StopShooting();
        }

        // if(cylinder.CanShoot()){
        //     candle.stop();
        // }else{
        //     candle.setBlinkGreen(); 
        // }

        maquinaLavar.DebugCylinder();
    }
    void StartShooting(){
         maquinaLavar.applyVelocity(ShooterConstants.kCylinderMultiplierRPM, 0);
        // isShooting = true;
    }
    void StopShooting(){
        maquinaLavar.applyVelocity(0, 0);
        isShooting = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
