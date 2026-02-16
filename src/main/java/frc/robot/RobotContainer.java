// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AcceleratorCommand;
import frc.robot.Commands.CapCommand;
import frc.robot.Commands.CatcherCommand;
import frc.robot.Commands.CylinderCommand;
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
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final Joystick controller = new Joystick(0);
  private final PS5Controller buttonController = new PS5Controller(0);
  private final Trigger button1 = new JoystickButton(controller, 6);
  private final CapSubsystem cap = new CapSubsystem();
  private final Cylinder cylinder = new Cylinder();
  private final Accelerator accelerator = new Accelerator();
  private final CatcherSubsystem catcher = new CatcherSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    Supplier<Double> axisZero = () -> -this.controller.getRawAxis(1);
    Supplier<Double> axisOne = () -> -this.controller.getRawAxis(0);
    Supplier<Double> axisTwo = () -> this.controller.getRawAxis(2);
    Supplier<Boolean> buttonSup = () -> this.buttonController.getOptionsButton();
    Supplier<Boolean> endAimSupplier = () -> this.buttonController.getR1ButtonPressed();
    Supplier<Boolean> ShooterSupplier = () -> this.buttonController.getL1ButtonPressed();
    Supplier<Boolean> cylinderSupplier = () -> this.buttonController.getL2ButtonPressed();
    Supplier<Boolean> catcherSupplier = () -> this.buttonController.getCrossButtonPressed();



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
    

    // shooter.setDefaultCommand(new ShooterCommand(shooter, ShooterSupplier));
    // cap.setDefaultCommand(new CapCommand(cap, ShooterSupplier));
    // cylinder.setDefaultCommand(new CylinderCommand(cylinder, cylinderSupplier));
    // accelerator.setDefaultCommand(new AcceleratorCommand(accelerator, ShooterSupplier));
    // catcher.setDefaultCommand(new CatcherCommand(catcherSupplier, catcher)); 




    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // private void configureBindings() {
    
  // }

  public Command getAutonomousCommand() {

    swerve.zeroHeading();

    return autoChooser.getSelected();
  }
  
}
