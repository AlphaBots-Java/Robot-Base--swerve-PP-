// package frc.robot.Commands;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.CapSubsystem;
// import frc.robot.Subsystems.LimeLightSubsystem;
// import frc.robot.Subsystems.ShooterSubsystem;

// public class CapCommand extends Command{
//     CapSubsystem cap;
//     Supplier<Boolean> capActivate;
//     Boolean canAngle = false;

//     public CapCommand(CapSubsystem cap, Supplier<Boolean> capActivate){
//         this.cap = cap;
//         this.capActivate = capActivate;
//     }

//     double calculateAngle(){
//         double shooterVelocity = 0.00266 * ShooterSubsystem.getShooterVelocityRPM();
//         double distanceX = LimeLightSubsystem.DistanceToTarget();
//         double time = 4; // The higher this value goes, the steeper the angle will be, and the ball will fly higher in the air

//         double angle = Math.acos(distanceX / (time * shooterVelocity));

//         return angle;
//     }

//     @Override
//     public void initialize(){
//         canAngle = false;
//     }

//     @Override
//     public void execute(){
//         SmartDashboard.putNumber("CapAngle", cap.getEncoderValue());
//         if(capActivate.get() && canAngle == false){
//             canAngle = true;
//             cap.setAngleDegrees(calculateAngle());
//         }else if (capActivate.get() && canAngle == false){
//             canAngle = false;
//             cap.setAngleDegrees(calculateAngle());;
//         }

        
//     }
// }
