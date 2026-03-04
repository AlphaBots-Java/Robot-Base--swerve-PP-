// package frc.robot.Commands;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.hardware.CANdle;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.PS5Controller;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.StartEndCommand;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Subsystems.LimeLightSubsystem;
// import frc.robot.Subsystems.ShooterSubsystem;
// import frc.robot.Subsystems.CandleSubsystem;

// public class ShooterCommand extends Command{
    
//     private final ShooterSubsystem shooter;
//     private final Supplier<Boolean> onButton;
//     public boolean isShooting = false;
//     private CandleSubsystem CANdle = new CandleSubsystem();

//     public ShooterCommand(ShooterSubsystem shooter, Supplier<Boolean> onButton){
//         this.shooter = shooter;
//         this.onButton = onButton;
//         shooter.applyVelocity(0, 0);
//         addRequirements(shooter);
//     }

//     @Override
//     public void initialize() {
//         isShooting = false;
//         shooter.applyVelocity(0, 0); // Garante que o shooter comece desligado
//     }

//     @Override
//     public void execute(){
//         if (onButton.get() && !isShooting) {
//             isShooting = !isShooting;
//             if(isShooting == true){
//                 shooter.applyVelocity(2900, 0);
//             }
//             if(isShooting == false){
//                 shooter.applyVelocity(0, 0);
//             }
//         }

//         SmartDashboard.putBoolean("isShootingShooter", onButton.get());
//     }
    
//     // void calculateAndSetSpeed(){
//     //     double distance = LimeLightSubsystem.DistanceToTarget();

//     //     if (distance > 300.0) {
//     //         // shooterSetPoint = shooter.getRpmForDistance(distance);
//     //         shooterSetPoint = 2900.0;

//     //     } else {
//     //         shooterSetPoint = 2900.0;
//     //     }
//     // }
    
//     @Override
//     public void end(boolean interrupted) {
//         shooter.applyVelocity(0, 0);
//     }
// }