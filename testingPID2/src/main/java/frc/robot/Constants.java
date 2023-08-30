// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve{

        public static final double stickDeadband = 0.1;
        public static final boolean invertGyro = false;


        /* Drivetrain COnstants */
        public static final double trackWidth = Units.inchesToMeters(21.73);
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter*Math.PI;
        
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); // TODO, L1, L2, or L3 6.75:1
        public static final double angleGearRatio = ((150.0 / 7.0)/1.0 /*12.8 / 1.0*/ ); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;
        /* TODO Arm Motor PID Values */
        public static final double sarmKS = 1;
        public static final double sarmKG = 1.0;
        public static final double sarmKV = 1.95;
        public static final double sarmKA = 0.06;
        public static final double sarmKP = 5;
        public static final double sarmKI = 0;
        public static final double sarmKD = 1;
        public static final double sarmKFF = 0;

        public static final double jarmKP = 0.01;
        public static final double jarmKI = 0.005;
        public static final double jarmKD = 0.0025;
        public static final double jarmKFF = 0;

        /* TODO tune Angle Motor PID Values */
        public static final double angleKP = 0.005;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.025;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.11563;//0.667;
        public static final double driveKV = 2.656;//2.44;
        public static final double driveKA = 0.276;//0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
            (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
        public static final int driveMotorID = 8;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 2;
       
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(203.75);//23.55
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
        public static final int driveMotorID = 6;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(149.15);//508.9);//328.9
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(40.0);//219.2
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
        public static final int driveMotorID = 2;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 4;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(337.02734375);//158.02734375
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        public static final class arm {

            public static final class Shoulder {
                public static final double Length = 34;
                public static final int rotMotorID = 9; 
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(8); //TODO
                public static final double gearRatio = 640.0; 
                /* TODO Arm Motor PID Values */
                public static final double Ks = 1;
                public static final double Kg = 1.0;
                public static final double Kv = 1.95;
                public static final double Ka = 0.06;
                public static final double Kp = 0.05;
                public static final double Ki = 0;
                public static final double Kd = 0;
                public static final double Kff = 0;
            }
            public static final class Joint {
                public static final double Length = 23.5;
                public static final int rotMotorID = 10;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(23); //TODO
                public static final double gearRatio = 300.0; 
                public static final double Ks = 1;
                public static final double Kg = 1.0;
                public static final double Kv = 1.95;
                public static final double Ka = 0.06;
                public static final double Kp = 0.01;
                public static final double Ki = 0;
                public static final double Kd = 0;
                public static final double Kff = 0;
            }
            public static final double[] INTAKE = new double[]{0, 82.46 + 5.3}; 
            public static final double[] MIDGOAL = new double[]{49.26 + 1.6 - 6.4 + 6.8, 61.97+ 5.3}; 
        public static final double[] HIGHGOAL = new double[]{86.58-3- 6.4 + 6.8, 130.2 /*86.94 + 1.6 + 5, 132.26 + 5.3*/}; 
            public static final double[] RETRACTED = new double[]{0, 0}; 
            public static final double[] SHELF = new double[]{81.65 + 1.6- 6.4 + 6.8, 154.2 + 5.3}; 

        }
        public static final class camera{
            public static final double Height = 0;
            //public static final double TargetHeight = 0;
            public static final double CameraPitch = 0;
            public static final Rotation3d camAngle = new Rotation3d(0, 0, 0/*1.571*/);
            public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0.2921), camAngle);
            public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
        }


    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.000000000000001;//0.2;//0.1;
        public static final double kPYController = 0.000000000000001;//0.2;// 0.1;
        public static final double kPThetaController = 7.5;
        public static final Translation2d GOALRIGHT = new Translation2d(1, -1);
        public static final Translation2d GOALLEFT = new Translation2d(1, 1);
        public static final Translation2d GOALMIDDLE = new Translation2d(1, 0);
        public static final PathConstraints constraints = new PathConstraints(kMaxAngularSpeedRadiansPerSecond, kMaxAccelerationMetersPerSecondSquared);
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
