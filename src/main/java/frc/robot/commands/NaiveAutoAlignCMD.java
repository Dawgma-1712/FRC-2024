package frc.robot.commands;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NaiveAutoAlignCMD extends Command{
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private PIDController rotationPID;
    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(Constants.AutoConstants.translationSlewRateAA);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.AutoConstants.strafeSlewRateAA);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.AutoConstants.rotationSlewRateAA);
    private int[] targetIds;
    private double targetAngle;

    public NaiveAutoAlignCMD(
        Swerve s_Swerve,
        DoubleSupplier translationSup,
        DoubleSupplier strafeSup,
        BooleanSupplier robotCentricSup,
        int[] targetIds) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.robotCentricSup = robotCentricSup;
        this.rotationPID = new PIDController(
            Constants.Swerve.rotationKP, 
            Constants.Swerve.rotationKI, 
            Constants.Swerve.rotationKD);
        this.targetIds = targetIds;
    }
    
    public double optimizeAngles(double targetAngle, double currentAngle){
        double outputAngle = currentAngle;
        while(outputAngle - targetAngle > 180){
            outputAngle -= 180;
        }
        while(outputAngle - targetAngle < -180){
            outputAngle += 180;
        }
        return outputAngle;
    }
    
    public void getTargetAngle(){
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
        LimelightHelpers.LimelightTarget_Fiducial[] llArr = llresults.targetingResults.targets_Fiducials;

        for(LimelightHelpers.LimelightTarget_Fiducial f :llArr){
            boolean contained = false;
            for(int i: this.targetIds){
                if(i==f.fiducialID) contained = true;
            }
            if(contained){
                this.targetAngle=s_Swerve.getYaw().getDegrees() + f.tx;
                return;
            }
        }
    }

    @Override
    public void execute() {
        getTargetAngle();
        // System.out.println(this.targetAngle);
        SmartDashboard.putNumber("currentAngle", s_Swerve.getYaw().getDegrees());
        SmartDashboard.putNumber("targetAngle", this.targetAngle);
        SmartDashboard.putNumber("optimizedAngle", optimizeAngles(this.targetAngle, s_Swerve.getYaw().getDegrees()));
        SmartDashboard.putNumber("rawPID", rotationPID.calculate(optimizeAngles(this.targetAngle, s_Swerve.getYaw().getDegrees())));
        
        /* Get Values, Deadband*/
        double translationVal =
            translationLimiter.calculate(
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
        double strafeVal =
            strafeLimiter.calculate(
                MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
        
        double rotationVal = rotationPID.calculate(optimizeAngles(this.targetAngle, s_Swerve.getYaw().getDegrees()));
        SmartDashboard.putNumber("rotationVal", rotationVal);
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            !robotCentricSup.getAsBoolean(),
            true);
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return false;
    }
}