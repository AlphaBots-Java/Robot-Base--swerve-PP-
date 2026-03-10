package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
        public static final double kDriveMotorGearRatio = 1 / 5.68;
        public static final double kTurningMotorGearRatio = 1 / 12.1;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(25);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(29.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // ---motorports---
        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 14;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackRightDriveMotorPort = 13;

        public static final int kFrontLeftTurningMotorPort = 21;
        public static final int kBackLeftTurningMotorPort = 24;
        public static final int kFrontRightTurningMotorPort = 22;
        public static final int kBackRightTurningMotorPort = 23;

        public static final InvertedValue kFrontLeftTurningEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kBackLeftTurningEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kFrontRightTurningEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kBackRightTurningEncoderReversed = InvertedValue.Clockwise_Positive;

        public static final InvertedValue kFrontLeftDriveEncoderReversed = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kBackLeftDriveEncoderReversed = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kFrontRightDriveEncoderReversed = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kBackRightDriveEncoderReversed = InvertedValue.CounterClockwise_Positive;

        // ---CANcoder Ports---

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 41;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 44;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 42;
        public static final int kBackRightDriveAbsoluteEncoderPort = 43;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
        
        // Caso essas constantes estejam mal calibradas, zere-as, coloque as rodas da swerve no zero, veja o numero que deu na smart dashboard
        // Apos isso, coloque aa swerve full pra frente (preferencialmente desreferenciada) e coloque a diferenca dos valores ai

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.614;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.8;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 8;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 40;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 0.5;
        public static final double kAngleDeadband = 0.05;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class ShooterConstants{
        public static final double kCapStep = 360/21; // angle generated for each axis spin
        // we don`t know the above yet

        public static final double CapMotorGearRatio = 100;

        public static final double kShooterLowSpeedRPM = 10;
        public static final double kShooterHighSpeedRPM = 10;
        
        public static final double kCylinderMultiplierRPM = 0.8;
        public static final double kAcceleratorMultiplierRPM = 0.8;

        public static final double kDistanceForHighSpeedMeters = 2;

        public static final double kCatcherRotationsToMM = 75 ; //75 mm pra uma volta do eixo
        public static final double kCatcherMaxExtensionMM = 80;
    }

    public static final class CandleConstants {
        public static final int kCandleId = 1; // Set this to your actual CANdle ID
        public static final int kLedCount = 8;  // Set this to the number of LEDs in your strip
    }
}