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

public class NaiveAutoAlignCMD extends Command{
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private PIDController rotationPID;
    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
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
            if(Arrays.asList(this.targetIds).contains(f.fiducialID)){
                System.out.println(f.tx);
                this.targetAngle=s_Swerve.getYaw().getDegrees() + f.tx;
                return;
            }
        }
    }

    @Override
    public void execute() {
        getTargetAngle();
        // System.out.println(this.targetAngle);
        /* Get Values, Deadband*/
        double translationVal =
            translationLimiter.calculate(
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
        double strafeVal =
            strafeLimiter.calculate(
                MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
        
        double rotationVal =
            rotationLimiter.calculate(
                MathUtil.applyDeadband(rotationPID.calculate(optimizeAngles(this.targetAngle, s_Swerve.getYaw().getDegrees())), Constants.Swerve.stickDeadband));

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