// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
// import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.LimeLightCommand;
import frc.robot.Commands.LimeLightCommandAuto;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final LimeLightSubsystem limelight = new LimeLightSubsystem();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Joystick controller = new Joystick(0);
  private final PS5Controller buttonController = new PS5Controller(0);
  private final Trigger button1 = new JoystickButton(controller, 6);


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


  }

  // private void configureBindings() {
    
  // }

  public Command getAutonomousCommand() {

    swerve.zeroHeading();

    return new PathPlannerAuto("bumpLowBar");
  }
  
}
