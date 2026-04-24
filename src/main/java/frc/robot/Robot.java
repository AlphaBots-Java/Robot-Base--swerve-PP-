// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import java.io.Console;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.CandleSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  CandleSubsystem CANdle = new CandleSubsystem();

  private final RobotContainer m_robotContainer;
  
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {
    m_Orchestra.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    CANdle.setBlinkBlue();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    CANdle.stop();
  }

  Orchestra m_Orchestra = new Orchestra();

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    // Vibra o controle ao iniciar o Teleop
  
}

  @Override
  public void teleopPeriodic() {
    // if(ps.getL2Button()){
    //   cylinder.applyVelocity(778, 0);
    //   accel.applyVelocity(1559, 0);
    //   shooter.applyVelocity(1000, 0);
    // }else{
    //   shooter.applyVelocity(0, 0);
    //   cylinder.applyVelocity(0, 0);
    //   accel.applyVelocity(0, 0);
    // }
    // if(ps.getTriangleButton()){
    //   cap.setAngleDegrees(-7);
    // }else{
    //   cap.setAngleDegrees(0);
    // }
    // shooter.DebugShooter();
    // accel.DebugAccel();
    // cylinder.DebugCylinder();
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
