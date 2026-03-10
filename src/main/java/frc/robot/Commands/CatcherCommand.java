// package frc.robot.Commands;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Subsystems.CatcherSubsystem;

// public class CatcherCommand extends Command{
//     Supplier<Boolean> catcherButton;
//     CatcherSubsystem catcher = new CatcherSubsystem();
//     Boolean isCatching = false;

//     public CatcherCommand(Supplier<Boolean> catcherButton, CatcherSubsystem catcher){
//         this.catcher = catcher;
//         this.catcherButton = catcherButton;
//     }

//     @Override
//     public void execute(){
//         if(catcherButton.get() && isCatching == false){
//             ExtendAndCatch();
//             isCatching = true;
//         }else if (catcherButton.get() && isCatching == true){
//             RetractAndStop();
//             isCatching = false;
//         }
//     }

//     void ExtendAndCatch(){
//         catcher.setCatcherExtenderMM(ShooterConstants.kCatcherMaxExtensionMM);
//         // catcher.setCatcherVelocity(0.5);
//     }

//     void RetractAndStop(){
//         catcher.setCatcherExtenderMM(0);
//         // catcher.setCatcherVelocity(0.0);
//     }
// }