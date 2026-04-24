package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class RobotContainer {

  // Subsystems
  private final LimeLightSubsystem limelight = new LimeLightSubsystem();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Cylinder cilindro = new Cylinder();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final CapSubsystem cap = new CapSubsystem();
  private final Accelerator accelerator = new Accelerator();
  private final CatcherSubsystem catcher = new CatcherSubsystem();

  // Controllers
  private final PS5Controller driver = new PS5Controller(0);
  private final PS5Controller copilot = new PS5Controller(1);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureAuto();
  }

  private void configureDefaultCommands() {

    Supplier<Double> axisX = () -> -driver.getLeftY();
    Supplier<Double> axisY = () -> -driver.getLeftX();
    Supplier<Double> rot = () -> driver.getRightX();
    Supplier<Boolean> fieldOriented = () -> driver.getOptionsButton();

    swerve.setDefaultCommand(
      new SwerveCommand(swerve, axisX, axisY, rot, fieldOriented)
    );

    // 🧠 UPDATED: catcher now stops cleanly
    catcher.setDefaultCommand(
      new RunCommand(() -> catcher.stop(), catcher)
    );

    accelerator.setDefaultCommand(
      new RunCommand(() -> accelerator.TurnOffAccelerator(), accelerator)
    );

    shooter.setDefaultCommand(
      new RunCommand(() -> shooter.TurnOffShooter(), shooter)
    );
  }

  private void configureBindings() {

    // AIM
    new JoystickButton(driver, 6).onTrue(
      new LimeLightCommand(
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> driver.getR1ButtonPressed(),
        limelight,
        () -> driver.getL2ButtonPressed(),
        swerve
      )
    );

    // SHOOT
    new JoystickButton(driver, 7).onTrue(
      Commands.sequence(
        new InstantCommand(() -> shooter.SetShooter()),
        new InstantCommand(() -> cap.RunExtender()),
        new InstantCommand(() -> accelerator.SetAccelerator())
      )
    );

    // CYLINDER / EXTENSION SEQUENCE (unchanged logic)
    new JoystickButton(driver, 5).onTrue(
      Commands.sequence(
        Commands.waitSeconds(.4),
        new InstantCommand(() -> catcher.toggleExtended(), catcher),
        Commands.waitSeconds(.4),
        new InstantCommand(() -> catcher.toggleExtended(), catcher),
        new InstantCommand(() -> catcher.setIntake())
      )
    );

    // =========================
    // COPILOT (UPDATED CATCHER)
    // =========================

    // EXTEND TOGGLE
    new JoystickButton(copilot, 1)
        .onTrue(new InstantCommand(() -> catcher.toggleExtended()));

    // DROP (hold)
    new JoystickButton(copilot, 7)
        .whileTrue(new RunCommand(() -> catcher.setExpel(), catcher))
        .onFalse(new InstantCommand(() -> catcher.stop()));

    // SUCK / INTAKE (hold)
    new JoystickButton(copilot, 8)
        .whileTrue(new RunCommand(() -> catcher.setIntake(), catcher))
        .onFalse(new InstantCommand(() -> catcher.stop()));
  }

  // ================= AUTO =================
  private void configureAuto() {

    NamedCommands.registerCommand("LimeLight-Oriented",
        new LimeLightCommandAuto(limelight, swerve));

    NamedCommands.registerCommand("ShooterState",
        new InstantCommand(() -> shooter.SetShooter()));

    NamedCommands.registerCommand("CapState",
        new InstantCommand(() -> cap.RunExtender()));

    NamedCommands.registerCommand("AccelState",
        new InstantCommand(() -> accelerator.SetAccelerator()));

    NamedCommands.registerCommand("CylinderState",
        new InstantCommand(() -> cilindro.SetCylinder()));

    NamedCommands.registerCommand("AxisState",
        new InstantCommand(() -> catcher.toggleCatch()));

    NamedCommands.registerCommand("ExtenderState",
        new InstantCommand(() -> catcher.toggleExtended()));

    NamedCommands.registerCommand("SubsystemsOFF",
        Commands.sequence(
          new InstantCommand(() -> shooter.TurnOffShooter()),
          new InstantCommand(() -> accelerator.TurnOffAccelerator()),
          new InstantCommand(() -> cilindro.TurnOffCylinder()),
          new InstantCommand(() -> catcher.stop()),
          new InstantCommand(() -> cap.RunExtender())
        )
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}