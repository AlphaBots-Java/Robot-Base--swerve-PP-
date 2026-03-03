package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.CapCommand;
import frc.robot.Commands.CatcherCommand;
import frc.robot.Commands.LimeLightCommand;
import frc.robot.Commands.LimeLightCommandAuto;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.Accelerator;
import frc.robot.Subsystems.CapSubsystem;
import frc.robot.Subsystems.CatcherSubsystem;
import frc.robot.Subsystems.Cylinder;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final LimeLightSubsystem limelight = new LimeLightSubsystem();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Joystick controller = new Joystick(0);
  private final PS5Controller buttonController = new PS5Controller(0);
  private final Trigger button1 = new JoystickButton(controller, 6);

  private final Trigger ShooterTrigger = new JoystickButton(controller, 5);
  private final Trigger CylinderTrigger = new JoystickButton(controller, 7);
  private final Cylinder cilindro = new Cylinder();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final CapSubsystem cap = new CapSubsystem();
  private final Accelerator accelerator = new Accelerator();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    Supplier<Double> axisZero = () -> -this.controller.getRawAxis(1);
    Supplier<Double> axisOne = () -> -this.controller.getRawAxis(0);
    Supplier<Double> axisTwo = () -> this.controller.getRawAxis(2);
    Supplier<Boolean> buttonSup = () -> this.buttonController.getOptionsButton();
    Supplier<Boolean> endAimSupplier = () -> this.buttonController.getR1ButtonPressed();



    NamedCommands.registerCommand("LimeLight-Oriented", new LimeLightCommandAuto(limelight, swerve));
    swerve.setDefaultCommand(new SwerveCommand(
      this.swerve,
      axisZero,
      axisOne,
      axisTwo,
      buttonSup
    ));

    button1.onTrue(new LimeLightCommand(axisZero,
                                        axisOne,
                                        endAimSupplier,
                                        limelight, 
                                        swerve));

    ShooterTrigger.onTrue(new InstantCommand(() -> {shooter.ActOrNotShooter();}));
    ShooterTrigger.onTrue(new InstantCommand(() -> {cap.RunExtender();}));
    ShooterTrigger.onTrue(new InstantCommand(() -> {accelerator.SetAccelerator();}));
    CylinderTrigger.onTrue(new InstantCommand(() -> {cilindro.SetCylinder();}));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    // swerve.zeroHeading();
    return autoChooser.getSelected();
  }

 
}
