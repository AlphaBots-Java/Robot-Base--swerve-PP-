package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.LimeLightCommand;
import frc.robot.Commands.LimeLightCommandAuto;
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
  private final Trigger AlignTrigger = new JoystickButton(controller, 6);
  private final Trigger CatcherTrigger = new JoystickButton(controller, 1);
  private final Trigger DropperTrigger = new JoystickButton(controller, 3);
  private final Trigger SuckerTrigger = new JoystickButton(controller, 2);
  private final Trigger CylinderTrigger = new JoystickButton(controller, 5);
  private final Trigger ShooterTrigger = new JoystickButton(controller, 7);
  private final Trigger TransferTrigger = new JoystickButton(buttonController, 14);
  private final Cylinder cilindro = new Cylinder();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final CapSubsystem cap = new CapSubsystem();
  private final Accelerator accelerator = new Accelerator();
  private final CatcherSubsystem catcher = new CatcherSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    Supplier<Double> axisZero = () -> -this.controller.getRawAxis(1);
    Supplier<Double> axisOne = () -> -this.controller.getRawAxis(0);
    Supplier<Double> axisTwo = () -> this.controller.getRawAxis(2);
    Supplier<Boolean> buttonSup = () -> this.buttonController.getOptionsButton();
    Supplier<Boolean> endAimSupplier = () -> this.buttonController.getR1ButtonPressed();
    BooleanSupplier isShootingSupplier = () -> !(ShooterSubsystem.m_currentSetpoint > 0);

    NamedCommands.registerCommand("LimeLight-Oriented", new LimeLightCommandAuto(limelight, swerve));
    NamedCommands.registerCommand("ShooterState", new InstantCommand(() -> {shooter.SetShooter();}));

    NamedCommands.registerCommand("CapState", new InstantCommand(() -> {cap.RunExtender();}));
    NamedCommands.registerCommand("AccelState", new InstantCommand(() -> {accelerator.SetAccelerator();}));
    NamedCommands.registerCommand("CylinderState", new InstantCommand(() -> {cilindro.SetCylinder();}));
    NamedCommands.registerCommand("AxisState", new InstantCommand(() -> {catcher.SetCatching();}));

    NamedCommands.registerCommand("ExtenderState", new InstantCommand(() -> {catcher.SetExtendedCREU();}));

    NamedCommands.registerCommand("SubsystemsOFF", Commands.sequence(
      new InstantCommand(() -> {shooter.TurnOffShooter();}),
      new InstantCommand(() -> {accelerator.TurnOffAccelerator();}),
      new InstantCommand(() -> {cilindro.TurnOffCylinder();}),
      new InstantCommand(() -> {catcher.SetCatchingFalse();}),
      new InstantCommand(() -> {cap.RunExtender();})
    ));


    swerve.setDefaultCommand(new SwerveCommand(
      this.swerve,
      axisZero,
      axisOne,
      axisTwo,
      buttonSup
    ));

    AlignTrigger.onTrue(new LimeLightCommand(axisZero,
                                        axisOne,
                                        endAimSupplier,
                                        limelight, 
                                        swerve));



    ShooterTrigger.onTrue(new InstantCommand(() -> {shooter.SetShooter();}));

    // creu
    CylinderTrigger.onTrue(
    Commands.sequence(
      Commands.waitSeconds(.2),
      Commands.runOnce(() -> catcher.SetExtendedCREU(), catcher),
      Commands.waitSeconds(.2),
      Commands.runOnce(() -> catcher.SetExtendedCREU(), catcher),
      new InstantCommand(() -> {catcher.SetCatchingTrue();})
      ).repeatedly().until(isShootingSupplier).andThen(Commands.sequence(new InstantCommand(() -> {catcher.SetCatchingFalse();}),
                                                                         new InstantCommand(() -> {catcher.SetExtendedTrue();})))
    );
    
    CylinderTrigger.onTrue(new InstantCommand(() -> {catcher.SetCatching();}));
    ShooterTrigger.onTrue(new InstantCommand(() -> {cap.RunExtender();}));
    ShooterTrigger.onTrue(new InstantCommand(() -> {accelerator.SetAccelerator();}));
    CylinderTrigger.onTrue(new InstantCommand(() -> {cilindro.SetCylinder();}));
    CatcherTrigger.onTrue(new InstantCommand(() -> {catcher.SetExtended();}));
    DropperTrigger.onTrue(new InstantCommand(() -> {catcher.setDropping();}));
    SuckerTrigger.onTrue(new InstantCommand(() -> {catcher.SetCatching();}));
    // TransferTrigger.onTrue(new InstantCommand(() -> {cap.ChangeAlignMode();}));
    


    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

 
}
