// // package frc.robot.Subsystems;

// // import com.ctre.phoenix6.hardware.CANcoder;
// // import com.ctre.phoenix6.configs.TalonFXConfiguration;
// // import com.ctre.phoenix6.hardware.TalonFX;

// // import edu.wpi.first.math.controller.PIDController;
// // import edu.wpi.first.wpilibj.DriverStation;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // import edu.wpi.first.wpilibj2.command.SubsystemBase;

// // // Positive Out extender
// // // positive shoots axis out the robot

// // public class CatcherSubsystem extends SubsystemBase{
// //     private final TalonFX krakenExtensor = new TalonFX(51, "BallSystemCAN");
// //     private final TalonFX krakenCatcher = new TalonFX(52, "BallSystemCAN");
// //     private final CANcoder catcherCANcoder = new CANcoder(50, "BallSystemCAN");
// //     boolean isExtended = false;
// //     int isCatching = 0;
// //     double newSpeed = 0;
// //     double clampedSetPoint = 0;
// //     double axisSpeedSetpoint = 0;

// //     private PIDController catcherController = new PIDController(2.7, 0.5, 0.1);
// //     private PIDController catcherController = new PIDController(2.7, 0.0, 0.1); // O termo Integral (I) foi zerado para evitar "windup"
// //     // private PIDController catcherController = new PIDController(0.5, 0, 0); Debug PID values
// //     private PIDController AxisController = new PIDController(0.03, 0, 0);

// //     public CatcherSubsystem(){
// //         TalonFXConfiguration catcherConfigs = new TalonFXConfiguration();
// //         catcherConfigs.CurrentLimits.StatorCurrentLimit = 40; // Limite de corrente do estator em Amperes. Ajuste conforme necessário.
// //         catcherConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
// //         krakenCatcher.getConfigurator().apply(catcherConfigs);

// //         isCatching = 0;
// //         isExtended = false;
// //         newSpeed = 0;
// //         clampedSetPoint = 0;
// //         axisSpeedSetpoint = 0;
// //         catcherController.setTolerance(0.05); // Define uma tolerância para o PID (ex: 0.05 rotações)
// //     }


// //     public void setCatcherExtenderMM(double setPoint){
// //         clampedSetPoint = setPoint;
// //         krakenExtensor.setVoltage(catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint)); 
// //         double calculatedVoltage = catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint);
// //         if (catcherController.atSetpoint()) {
// //             krakenExtensor.setVoltage(0.0); // Desliga o motor se estiver dentro da tolerância do setpoint
// //         } else {
// //             krakenExtensor.setVoltage(calculatedVoltage);
// //         }
// //         // if(isCatching == 2 && catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint) < 0.5){
// //         //     isCatching = 0;
// //         // }
// //     }

// //     // public void setCatcherExtenderCREU(double setPoint){
// //     //     new Thread(() -> {
// //     //         clampedSetPoint = setPoint;
// //     //         while(!(catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint) < 0.5 && catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint) > -0.5)){
// //     //             krakenExtensor.setVoltage(catcherController.calculate(-catcherCANcoder.getPosition().getValueAsDouble(), clampedSetPoint)); 
// //     //         }
// //     //         krakenExtensor.setVoltage(0);
// //     //         return;
// //     //     }).start(); 
// //     // }


// //     public void SetExtendedCREU(){
// //         // if(isExtended == false){
// //         //     setCatcherExtenderCREU(2.6);
// //         // }
// //         // if(isExtended == true){
// //         //     setCatcherExtenderCREU(0);
// //         // }
// //         isExtended = !isExtended;
// //     }

// //     public void SetExtended(){
// //         // if(isExtended == false){
// //         //     setCatcherExtenderMM(2.6);
// //         //     // setExpellingTrue();
// //         // }
// //         // if(isExtended == true){
// //         //     setCatcherExtenderMM(0);
// //         // }
// //         isExtended = !isExtended;
// //     }



// //     public void SetExtendedTrue(){
// //         // setCatcherExtenderMM(2.6);
// //         isExtended = true;
// //     }

// //     public void SetCatching(){
// //         if(isCatching == 0 || isCatching == 2){
// //             isCatching = 1;
// //         }
// //         else{
// //             isCatching = 0;
// //         }
// //     }
// //     public void SetCatchingTrue(){
// //         isCatching = 1;
// //     }

// //     public void SetCatchingFalse(){
// //         isCatching = 0;
// //     }

// //     public void setExpellingTrue(){
// //         isCatching = 2;
// //     }

// //     public void setDropping(){
// //         if(isCatching == 0 || isCatching == 1){
// //             isCatching = 2;
// //         }
// //         else{
// //             isCatching = 0;
// //         }
// //     }


// //     void DebugCatcher(){
// //         SmartDashboard.putNumber("Catcher CanCoder", catcherCANcoder.getPosition().getValueAsDouble());
// //         SmartDashboard.putNumber("extenderSetpoint", clampedSetPoint);

// //     }

// //     @Override
// //     public void periodic(){
// //         DebugCatcher();
// //         if(DriverStation.isEnabled()){
// //             if(isExtended == false){
// //                 setCatcherExtenderMM(0.9);
// //             }else if (isExtended == true){
// //                 setCatcherExtenderMM(2.3);
// //             }

// //             if(isCatching == 1){
// //                 // setCatcherVelocity(CatcherSpeed);
// //                 krakenCatcher.set(-0.9);
// //             }else if(isCatching == 2){
// //                 // setCatcherVelocity(-CatcherSpeed);
// //                 krakenCatcher.set(0.9);
// //             }
// //             else if(isCatching == 0){
// //                 // setCatcherVelocity(0.0);
// //                 krakenCatcher.set(0.0);
// //             }
// //         }else{
// //             isCatching = 0;
// //             krakenCatcher.set(0);
// //             // setCatcherVelocity(0.0);
// //         }
// //         DebugCatcher();
        
// //     }
// // }

// package frc.robot.Subsystems;

// import java.util.function.Supplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.PS5Controller;
// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.OIConstants;
// import frc.robot.Subsystems.SwerveSubsystem;

// public class SwerveCommand extends Command {

//     private SwerveSubsystem swerveSubsystem;
//     private Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
//     private Supplier<Boolean> fieldOrientedFunction;

//     private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

//     private PS5Controller controller = new PS5Controller(0);

//     public SwerveCommand(
//             SwerveSubsystem swerveSubsystem,
//             Supplier<Double> xSpdFunction,
//             Supplier<Double> ySpdFunction,
//             Supplier<Double> turningSpdFunction,
//             Supplier<Boolean> fieldOrientedFunction) {

//         this.swerveSubsystem = swerveSubsystem;
//         this.xSpdFunction = xSpdFunction;
//         this.ySpdFunction = ySpdFunction;
//         this.turningSpdFunction = turningSpdFunction;
//         this.fieldOrientedFunction = fieldOrientedFunction;

//         this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
//         this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
//         this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

//         addRequirements(swerveSubsystem);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {

//         double xSpeed;
//         double ySpeed;
//         double turningSpeed;

//         if (controller.getPSButton()) {

//             xSpeed = 0;
//             ySpeed = -0.1;
//             turningSpeed = turningSpdFunction.get();

//         } else {

//             xSpeed = xSpdFunction.get();
//             ySpeed = ySpdFunction.get();
//             turningSpeed = -turningSpdFunction.get();
//         }

//         OutputToWheels(xSpeed, ySpeed, turningSpeed);
//     }

//     public void OutputToWheels(double xspd, double yspd, double turningspd) {

//         // 2. Apply deadband
//         xspd = Math.abs(xspd) > OIConstants.kDeadband ? xspd : 0.0;
//         yspd = Math.abs(yspd) > OIConstants.kDeadband ? yspd : 0.0;
//         turningspd = Math.abs(turningspd) > OIConstants.kDeadband ? turningspd : 0.0;

//         // 3. Make the driving smoother
//         xspd = xLimiter.calculate(xspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//         yspd = yLimiter.calculate(yspd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//         turningspd = turningLimiter.calculate(turningspd)
//                 * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

//         // 4. Construct desired chassis speeds
//         ChassisSpeeds chassisSpeeds;

//         // if (fieldOrientedFunction.get()) {
//         chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                 xspd,
//                 yspd,
//                 turningspd,
//                 swerveSubsystem.getRotation2d());
//         // } else {
//         //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//         //             xSpeed,
//         //             -ySpeed,
//         //             turningSpeed,
//         //             swerveSubsystem.getRotation2d());
//         // }

//         if (fieldOrientedFunction.get()) {
//             swerveSubsystem.zeroHeading();
//         }

//         // 5. Convert chassis speeds to individual module states
//         SwerveModuleState[] moduleStates =
//                 DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

//         // 6. Output each module states to wheels
//         swerveSubsystem.setModuleStates(moduleStates);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerveSubsystem.stopModules();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }