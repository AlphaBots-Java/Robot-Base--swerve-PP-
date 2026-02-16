package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CapSubsystem;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class CapCommand extends Command{
    CapSubsystem cap;
    public CapCommand(CapSubsystem cap){
        this.cap = cap;
    }

    double calculateAngle(){
        double shooterVelocity = 0.00266 * ShooterSubsystem.getShooterVelocityRPM();
        double distanceX = LimeLightSubsystem.DistanceToTarget();
        double distanceY = 4; //Botar a altura do hub aqui
        double gravity = 9.8;
        
        double tan2theta = (2*gravity * distanceY)/(Math.pow(shooterVelocity, 2) - (gravity * 2* distanceX));
        double angle = Math.atan(tan2theta)/2;

        return angle;
    }
}
