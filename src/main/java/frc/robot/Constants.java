package frc.robot;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.05;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0);
    public static final double angleGearRatio = (21.43 / 1.0);

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

    /* Angle Motor PID Values */
    public static final double angleKP = 0.03;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    // Unused - had to determine individual PID values

    /* Drive Motor PID Values */
    public static final double driveKP = 0.3;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.5;
    public static final double driveKFF = 0.0;

    /* Swerve Autonomous PID Values & Configuration */
    public static final double translationKP = 1.0;
    public static final double translationKI = 0.0;
    public static final double translationKD = 6.0;
    public static final double rotationKP = 2.0;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.0;
    public static final double maxModuleSpeed = 2.0;
    public static final double driveBaseRadius = 0.8;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static double maxSpeed = 1; // meters per second
    public static final double maxAngularVelocity = 4;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 9;

      //Angle Motor PID Values
      public static final double angleKP = 0.05;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;
      public static final double angleKFF = 0.0;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(354.9);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleKP, angleKI, angleKD, angleKFF, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 6;

      //Angle Motor PID Values
      public static final double angleKP = 0.051;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;
      public static final double angleKFF = 0.0;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(15.7);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleKP, angleKI, angleKD, angleKFF, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 12;

      //Angle Motor PID Values
      public static final double angleKP = 0.01;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;
      public static final double angleKFF = 0.0;
      
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(60.2);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleKP, angleKI, angleKD, angleKFF, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID =2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 3;

      //Angle Motor PID Values
      public static final double angleKP = 0.01;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;
      public static final double angleKFF = 0.0;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(229.6);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleKP, angleKI, angleKD, angleKFF, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 0;
    public static final double kPThetaController = 4;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class OperatorConstants{
    public static final Map<String, Double> armExtendPresets = Map.ofEntries(
      Map.entry("coneMid", 37.0),
      Map.entry("coneHigh", 200.0),
      Map.entry("cubeMid", 30.0),
      Map.entry("cubeHigh", 200.0),
      Map.entry("stow", 0.0),
      Map.entry("substation", 120.0),
      Map.entry("ground", 0.0));
    public static final Map<String, Double> armRaisePresets = Map.ofEntries(
      Map.entry("coneMid", 16.8),
      Map.entry("coneHigh", 16.0),
      Map.entry("cubeMid", 18.0),
      Map.entry("cubeHigh", 17.0),
      Map.entry("stow", 0.0),
      Map.entry("substation", 16.2),
      Map.entry("ground", 28.5));
    public static final int OperatorControllerPort = 1;
    public static final int DriverControllerPort = 0;
    public static final int OperatorRaise = 5;
    public static final int OperatorExtend = 1;

    public static final int DriverYAxis = 1;
    public static final int DriverXAxis = 0;
    public static final int kDriverRotAxis = 2;
    public static final int DriverFieldOrientedButton = 3;

    public static final double deadband = 0.1;

    public static final int idsLength = 16;

    //Near station
    //Far station
    //Amp
    //Center speaker
    //Right speaker
    //Far stage
    //Left stage
    //Right stage
    public static final Double[] id_Blue = {1.0, 2.0, 6.0, 7.0, 8.0, 14.0, 15.0, 16.0};
    public static final Double[] id_Red = {10.0, 9.0, 5.0, 4.0, 3.0, 13.0, 12.0, 11.0};

    public static final double yDisplace = 38;
    public static final double llAngle = 26;

    public static final double[] aprilTagX = {593.68, 637.21, 652.73, 652.73, 578.77, 72.5, -1.5, -1.5, 14.02, 57.54, 468.69, 468.69, 441.74, 209.48, 182.73, 182.73};
    public static final double[] goalX = {-12, -12, -12, -12, -12, -12, 12, 12, -12, -12, -12, -12, -12, -12, -12, -12};
    public static final double[] aprilTagY = {9.68, 34.79, 196.17, 218.42, 323, 323, 218.42, 196.17, 34.79, 9.68, 146.19, 177.1, 161.62, 161.62, 177.1, 146.19};
    public static final double[] aprilTagZ = {53.38, 53.38, 57.13, 57.13, 53.38, 53.38, 57.13, 57.13, 53.38, 53.38, 52, 52, 52, 52, 52, 52};
    public static final double[] aprilTagYaw = {120, 120, 180, 180, 270, 270, 0, 0, 60, 60, 300, 60, 180, 0, 120, 240};
  }
}