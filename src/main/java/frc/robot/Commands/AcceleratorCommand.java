// package frc.robot.Commands;
// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Subsystems.Accelerator;
// import frc.robot.Subsystems.LimeLightSubsystem;

// public class AcceleratorCommand extends Command{
    
//     Accelerator accelerator = new Accelerator();
//     Supplier<Boolean> OnButton;
//     Boolean isShooting;
//     static Double acceleratorSetPoint;

//     public AcceleratorCommand(Accelerator accelerator ,Supplier<Boolean> OnButton){
//         this.accelerator = accelerator;
//         this.OnButton = OnButton;
//     }

//     @Override
//     public void initialize() {
//         isShooting = false;
//         acceleratorSetPoint = 0.0;
//     }

//     @Override
//     public void execute(){
//         calculateSpeed();

//         if(OnButton.get() && isShooting == false){
//             accelerator.applyVelocity(acceleratorSetPoint, 0);
//             isShooting = true;
//         }else if (OnButton.get() && isShooting == true){
//             accelerator.applyVelocity(0, 0);
//             isShooting = false;
//         }
//     }

//     void calculateSpeed(){
//         if (LimeLightSubsystem.DistanceToTarget() < ShooterConstants.kDistanceForHighSpeedMeters){
//             acceleratorSetPoint = ShooterConstants.kShooterLowSpeedRPM;
//         }else{
//             acceleratorSetPoint = ShooterConstants.kShooterHighSpeedRPM;
//         }
//     }

//     public static double getAcceleratorSetPoint(){
//         return  acceleratorSetPoint;
//     }
// }
