package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Cylinder;

public class CylinderCommand extends Command{
    
    Cylinder cylinder = new Cylinder();
    Supplier<Boolean> OnButton;
    Boolean isShooting;

    public CylinderCommand(Cylinder cylinder ,Supplier<Boolean> OnButton){
        this.cylinder = cylinder;
        this.OnButton = OnButton;
    }

    @Override
    public void initialize() {
        isShooting = false;
    }

    @Override
    public void execute(){

        if(OnButton.get() && isShooting == false){
            cylinder.applyVelocity(ShooterConstants.kCylinderSpeedRPM, 0);
            isShooting = true;

        }else if (OnButton.get() && isShooting == true){
            cylinder.applyVelocity(0, 0);
            isShooting = false;
        }
    }
}
