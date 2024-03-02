package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.LimelightHelpers;

import frc.robot.Constants;
import frc.robot.Debug;

public class Swerve extends SubsystemBase {
  //private final Pigeon2 gyro;
  private final AHRS gyro2 = new AHRS(SerialPort.Port.kUSB1);

  private SwerveDrivePoseEstimator swervePoseEstimator;
  private SwerveModulePosition[] positions = new SwerveModulePosition[4];

  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private boolean locked;

  private double percentSpeed;

  public Swerve() {
    //gyro = new Pigeon2(Constants.Swerve.pigeonID);
    //gyro.configFactoryDefault();
    zeroGyro();

    percentSpeed = 1;
    locked = false;
      // new SwerveModule[] {
      //     new SwerveModule(0, Constants.Swerve.Mod0.constants, 
      //     0.03, 0.0, 0.15,
      //     0.3, 0, 0),
      //     new SwerveModule(1, Constants.Swerve.Mod1.constants, 
      //     0.022, 0.0, 0.11,
      //     1.5, 0, 0),
      //     new SwerveModule(2, Constants.Swerve.Mod2.constants, 
      //     0.026, 0.0, 0.12,
      //     0.3, 0, 0),
      //     new SwerveModule(3, Constants.Swerve.Mod3.constants, 
      //     0.023, 0.0, 0.11,
      //     0.3, 0, 0)
      //   };
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants, 0.03, 0.0, 0.15),
          new SwerveModule(1, Constants.Swerve.Mod1.constants, 0.022, 0.0, 0.11),
          new SwerveModule(2, Constants.Swerve.Mod2.constants, 0.026, 0.0, 0.12),
          new SwerveModule(3, Constants.Swerve.Mod3.constants, 0.023, 0.0, 0.11)
        };
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    // need to add a defualt
    swervePoseEstimator = new SwerveDrivePoseEstimator(
          Constants.Swerve.swerveKinematics, 
          getYaw(),
          positions,
          new Pose2d());
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    //Path Planner - AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getSpeed,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.Swerve.translationKP, Constants.Swerve.translationKI, Constants.Swerve.translationKD),
        new PIDConstants(Constants.Swerve.rotationKP, Constants.Swerve.rotationKI, Constants.Swerve.rotationKD),
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.Swerve.wheelBase * Math.sqrt(2)/2,
        new ReplanningConfig()
        ),
        () -> {
            var alliance = DriverStation.getAlliance();
            if(alliance.isPresent()){
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SmartDashboard.putNumber("rotation", rotation);
        SmartDashboard.putNumber("XSpeed", translation.getX());
        SmartDashboard.putNumber("YSpeed", translation.getY());
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    //Debug.log(5, "Drive locked" + Boolean.toString(locked));
    if(locked){
        swerveModuleStates[0] = new SwerveModuleState(0, new Rotation2d(Math.PI/4.0));
        swerveModuleStates[1] = new SwerveModuleState(0, new Rotation2d(-Math.PI/4.0));
        swerveModuleStates[2] = new SwerveModuleState(0, new Rotation2d(-Math.PI/4.0));    
        swerveModuleStates[3] = new SwerveModuleState(0, new Rotation2d(Math.PI/4.0));
    }
    else{
        swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    }
    for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false/*isOpenLoop */, locked);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false, locked);
    }
  }

  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    swervePoseEstimator.resetPosition(getYaw(), positions, pose);
  }

  public void resetEncoders() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public boolean getLockedState(){
    return locked;
  }

  public void setLock(boolean locked){
    this.locked = locked;
  }

  public void stopModules(){
    for(SwerveModule mod : mSwerveMods){
        mod.stop();
    }
  }

  public void setSpeed(double speed){
    Constants.Swerve.maxSpeed = speed;
    //this.percentSpeed = speed;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    //gyro.setYaw(0);
    gyro2.reset();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro2.getYaw())
        : Rotation2d.fromDegrees(gyro2.getYaw());
  }

  @Override
  public void periodic() {
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), positions);

    // if(LimelightHelpers.getTV("")){
    //   double latency = LimelightHelpers.getLatency_Pipeline("") + LimelightHelpers.getLatency_Capture("");
    //   double timestamp = Timer.getFPGATimestamp()-latency;
    //   swervePoseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(""), timestamp);
    // }
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    SmartDashboard.putBoolean("Lock?", locked);
    SmartDashboard.putNumber("FieldX", getPose().getX());
    SmartDashboard.putNumber("FieldY", getPose().getY());
  }

  //Path Planner - AutoBuilder
  public ChassisSpeeds getSpeed(){
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    
    ChassisSpeeds targetSpeeds = robotRelativeSpeeds;//ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
}