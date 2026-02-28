
# FRC 1860 Robot Project Documentation

## 1. Overview Section

**Purpose:** This software project provides the comprehensive control architecture for 1860 robot. It operates a Kraken/TalonFX-powered swerve drive, a vision-aligned shooting superstructure, and autonomous path-following capabilities using PathPlanner.

**Key Features:**

* **Swerve Drivetrain Architecture:** Implements a holonomic drive system using four independently controlled swerve modules. Each module uses CTRE TalonFX motors for drive and steering, coupled with CANcoders for absolute azimuth positioning.
* **Superstructure Control:** Manages a complex payload delivery system divided into several subsystems: `Shooter`, `Accelerator`, `Cylinder` (indexer), `Cap` (adjustable hood), and `Catcher` (intake).
* **Dynamic Vision Targeting:** Integrates `LimelightHelpers` to parse NetworkTables data. The robot calculates its distance to the target using trigonometric functions and automatically adjusts the shooter RPM and hood angle dynamically.
* **PathPlanner Autonomous Integration:** Utilizes `AutoBuilder` and `PPHolonomicDriveController` to follow pre-mapped JSON paths (`.path` and `.auto` files) with PID-controlled trajectory correction.
* **Audio Feedback:** Utilizes CTRE's `Orchestra` class to play "up.chrp" through the swerve drive motors.

**When to Use:**

* Deploying code to the RoboRIO for teleoperated control and autonomous routines.
* Utilizing dynamic distance-based calculation for firing projectiles into a speaker/hub.
* Serving as a reference for integrating WPILib kinematics, CTRE Phoenix 6 APIs, and PathPlanner.

---

## 2. README Section

### Installation Instructions

1. **WPILib Environment:** Install the latest WPILib VS Code environment for the current FRC season.
2. **Vendor Dependencies:** You must install the following vendor libraries (via `Manage Vendor Libraries` in WPILib VS Code):
* CTRE Phoenix 6 (for TalonFX, CANcoder, Pigeon2)
* PathPlannerLib (for autonomous routines)


3. **Repository Clone:** Clone this project repository into your local workspace.
4. **Compilation:** Run the WPILib "Build Robot Code" command.

### Quick Start Guide

1. Turn on the robot and connect to its radio network or tether directly via USB to the RoboRIO.
2. Open the WPILib Command Palette and select **Deploy Robot Code**.
3. Open the FRC Driver Station.
4. Connect a generic Joystick to Port 0 and a PS5 Controller to Port 0 (or appropriately map them according to `RobotContainer.java`).
5. Enable the robot in **Teleoperated** mode. The left and right joysticks will control translation and rotation, respectively.

### Configuration Options

The `Constants.java` file is the central configuration hub. Key parameters include:

* **DriveConstants:** Physical dimensions (`kTrackWidth`, `kWheelBase`), CAN IDs (`kFrontLeftDriveMotorPort`, etc.), and absolute encoder offsets.
* *Calibration Note:* If modules fight each other, you must align all wheels perfectly forward, read the raw absolute encoder values via SmartDashboard, and input those offsets into `kFrontLeftDriveAbsoluteEncoderOffsetRad` (and corresponding variables).


* **ShooterConstants:** Defines RPM states (`kShooterLowSpeedRPM`, `kShooterHighSpeedRPM`) and the distance threshold (`kDistanceForHighSpeedMeters`) that toggles the state.
* **AutoConstants:** Defines maximum speeds, accelerations, and the PID gains (`kPXController`, `kPThetaController`) for PathPlanner trajectory following.

### Troubleshooting Common Issues

* **Swerve Modules Spinning Erratically:** Check the `absoluteEncoderReversed` booleans and the offset values in `Constants.java`. A misconfigured offset causes the PID controller to target the wrong angle.
* **Shooter Will Not Fire:** The `CylinderCommand` relies on a safety check (`cylinder.CanShoot()`). If the Limelight is not providing accurate distance data, or if the motors have not reached their target RPMs, the indexer will not feed the ball.
* **Robot Drives Incorrectly in Auto:** Verify that the Pigeon2 gyro is properly zeroed at the start of the match and that the `PathPlanner` settings match the robot's physical track width and wheel radius.

---

## 3. API Documentation

The following details the core public components of the control system.

```java
/**
 * @description Calculates the horizontal distance from the Limelight lens to the target base.
 * @returns {double} The distance to the target in centimeters.
 * @throws {ArithmeticException} If the Limelight angle results in a tangent of zero (divide by zero).
 * @example
 * double targetDist = LimeLightSubsystem.DistanceToTarget();
 */
public static double DistanceToTarget()

```

> **Behavior Note:** This method calculates distance using the `ty` (vertical offset) angle from the Limelight and predefined mount heights. This is expected behavior, not guaranteed, as an obstructed camera or uncalibrated crosshair will yield inaccurate data.

```java
/**
 * @description Evaluates whether the shooting sequence can safely proceed by verifying the velocities of the accelerator and shooter wheels.
 * @returns {boolean} True if the motors have reached their target RPMs and the velocity hierarchy is safe; otherwise false.
 * @example
 * if (cylinder.CanShoot()) {
 * StartShooting();
 * }
 */
public boolean CanShoot()

```

> **Behavior Note:** This aims to avoid jamming the mechanism by verifying the shooter spins faster than the accelerator. This is expected behavior, not guaranteed, as physical friction or voltage drops can still cause jams.

```java
/**
 * @description Commands the swerve drivetrain to move based on desired chassis speeds relative to the robot's coordinate frame.
 * @param {ChassisSpeeds} Mspeeds - The target X, Y, and Theta velocities.
 * @example
 * swerveSubsystem.driveRobotRelative(new ChassisSpeeds(1.5, 0.0, 0.0));
 */
public void driveRobotRelative(ChassisSpeeds Mspeeds)

```

```java
/**
 * @description Takes an array of desired swerve module states, desaturates the wheel speeds to respect physical limits, and applies the states to the hardware.
 * @param {SwerveModuleState[]} desiredStates - Array containing speed and angle targets for all 4 modules.
 * @example
 * swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
 */
public void setModuleStates(SwerveModuleState[] desiredStates)

```

---

## 4. Usage Examples

### Example 1: Basic Subsystem Command Implementation

Subsystems are linked to commands via `RobotContainer`. The following demonstrates how the `ShooterCommand` is mapped to a PS5 controller button.

```java
// Inside RobotContainer.java constructor
Supplier<Boolean> shooterSupplier = () -> this.buttonController.getL1ButtonPressed();

// The command continuously checks the supplier state to toggle the shooter motors
shooter.setDefaultCommand(new ShooterCommand(shooter, shooterSupplier));

```

### Example 2: Dynamic Vision-Based Aiming

To aim the robot at a target while allowing the driver to maintain translation control, the `LimeLightCommand` is used. This overrides the rotational input from the joystick with a PID controller evaluating the Limelight's horizontal offset (`tx`).

```java
// Inside RobotContainer.java constructor
button1.onTrue(new LimeLightCommand(
    axisZero,            // Forward/backward translation from driver
    axisOne,             // Left/right translation from driver
    endAimSupplier,      // Button to exit aiming mode
    limelight, 
    swerve
));

```

> **Behavior Note:** When the button is pressed, the robot is designed to snap its rotation to center the vision target while still sliding sideways or forward based on stick input. This is expected behavior, not guaranteed, as it relies on proper PID tuning in `LimeLightCommand`.

### Example 3: Error Handling and Safety Gates

The indexer (`Cylinder`) is responsible for pushing the game piece into the shooter. The code employs a velocity-checking pattern to verify the system is ready.

```java
// Inside CylinderCommand.java execute()
@Override
public void execute(){
    // Only fires if the button is pressed, it is not already shooting, AND CanShoot() passes
    if (OnButton.get() && isShooting == false && cylinder.CanShoot()){
        StartShooting();
    }
    if (OnButton.get() && isShooting == true){
        StopShooting();
    }
}

```

---

## 5. Inline Code Explanation

Below is a detailed, line-by-line breakdown of the `CapCommand.java` file, which handles the dynamic angling of the shooter hood.

```java
package frc.robot.Commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CapSubsystem;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class CapCommand extends Command{
    CapSubsystem cap;
    Supplier<Boolean> capActivate;
    Boolean canAngle = false;

    // Constructor injecting the subsystem and the trigger condition
    public CapCommand(CapSubsystem cap, Supplier<Boolean> capActivate){
        this.cap = cap;
        this.capActivate = capActivate;
    }

    /**
     * Calculates the required hood angle based on projectile physics.
     * This relies on a simplified kinematic model where time to target 
     * is approximated by a constant '4', and velocity is derived from RPM.
     * This is expected behavior, not guaranteed to hit the target if air resistance is high.
     */
    double calculateAngle(){
        // Convert shooter RPM to a linear surface velocity (m/s)
        double shooterVelocity = 0.00266 * ShooterSubsystem.getShooterVelocityRPM();
        
        // Get the horizontal distance to the target using vision
        double distanceX = LimeLightSubsystem.DistanceToTarget();
        
        // 'time' acts as a tuning constant to adjust the steepness of the arc
        double time = 4; 

        // Inverse cosine of (distance / (time * velocity)) yields the optimal angle in radians
        // V_x = V * cos(theta), therefore distance = V * cos(theta) * time
        double angle = Math.acos(distanceX / (time * shooterVelocity));

        return angle;
    }

    @Override
    public void initialize(){
        canAngle = false;
    }

    @Override
    public void execute(){
        // State machine pattern: toggles the hood position based on button input
        if(capActivate.get() && canAngle == false){
            canAngle = true;
            // Apply the dynamically calculated angle to the subsystem
            cap.setCapPositionDegrees(calculateAngle());
        }else if (capActivate.get() && canAngle == false){ // [Speculation] Logic flaw: condition matches the "if" statement. Likely meant `canAngle == true`.
            canAngle = false;
            // Return to zero position when deactivated
            cap.setCapPositionDegrees(0);
        }
    }
}

```
