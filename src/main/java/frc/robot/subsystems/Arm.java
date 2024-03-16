package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase{

    private final CANSparkMax raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private final RelativeEncoder raiseEncoder1 = raiseMotor1.getEncoder();
    private final RelativeEncoder raiseEncoder2 = raiseMotor2.getEncoder();
    private DigitalInput limitSwitchTop = new DigitalInput(0);
    private DigitalInput limitSwitchBottom = new DigitalInput(1);

    private final PIDController armRaisePID1 = new PIDController(Constants.EndEffectorConstants.armKP, Constants.EndEffectorConstants.armKI, Constants.EndEffectorConstants.armKD);
    private final PIDController armRaisePID2 = new PIDController(Constants.EndEffectorConstants.armKP, Constants.EndEffectorConstants.armKI, Constants.EndEffectorConstants.armKD);
    private Rotation2d targetAngle;
    private boolean positionControl = false;

    private double speed1, speed2 = 0;

    private ShuffleboardTab armTab = Shuffleboard.getTab("Arm Control");
    private GenericEntry angelChooser = armTab.add("targetPosition", Constants.OperatorConstants.armStartingPosition.getDegrees()).getEntry();

    private GenericEntry KPChooser = armTab.addPersistent("arm kp", 0).getEntry();
    private GenericEntry KIChooser = armTab.addPersistent("arm ki", 0).getEntry();
    private GenericEntry KDChooser = armTab.addPersistent("arm kd", 0).getEntry();

    private GenericEntry levelVoltageChooser = armTab.addPersistent("level voltage", 0).getEntry();

    private GenericEntry positionControlChooser = armTab.add("position control", false).getEntry();

    private GenericEntry targetAngleDisplay = armTab.add("Target Arm Angle", 0).getEntry();
    private GenericEntry currentAngleDisplay = armTab.add("Current Arm Angle",0).getEntry();
    private GenericEntry topSwitchDisplay = armTab.add("Top Limit Switch", false).getEntry();
    private GenericEntry bottomSwitchDisplay  =armTab.add("Bottom Limit Switch", false).getEntry();

    public Arm(){
        setIdle();
        stop();
        raiseMotor2.setInverted(true);
        resetEncoders(Constants.OperatorConstants.armStartingPosition);
        targetAngle = getAngle();
    }

    public void periodic(){
        if(positionControlChooser.getBoolean(false)){
            setTargetPosition(Rotation2d.fromDegrees(angelChooser.getDouble(getAngle().getDegrees())));
            armRaisePID1.setP(KPChooser.getDouble(armRaisePID1.getP()));
            armRaisePID1.setI(KIChooser.getDouble(armRaisePID1.getI()));
            armRaisePID1.setD(KDChooser.getDouble(armRaisePID1.getD()));
            armRaisePID2.setP(KPChooser.getDouble(armRaisePID2.getP()));
            armRaisePID2.setI(KIChooser.getDouble(armRaisePID2.getI()));
            armRaisePID2.setD(KDChooser.getDouble(armRaisePID2.getD()));
            Constants.EndEffectorConstants.armLevelVoltage = levelVoltageChooser.getDouble(Constants.EndEffectorConstants.armLevelVoltage);
        }
        
        if(positionControl){
            speed1 = armRaisePID1.calculate(getAngle1().getDegrees(), targetAngle.getDegrees()) + calculateFeedForward(getAngle1());
            speed2 = armRaisePID2.calculate(getAngle2().getDegrees(), targetAngle.getDegrees()) + calculateFeedForward(getAngle2());
        }
        
        speed1 = MathUtil.clamp(speed1, Constants.EndEffectorConstants.maxArmSpeedDown, Constants.EndEffectorConstants.maxArmSpeedUp);
        speed2 = MathUtil.clamp(speed2, Constants.EndEffectorConstants.maxArmSpeedDown, Constants.EndEffectorConstants.maxArmSpeedUp);

        targetAngleDisplay.setDouble(targetAngle.getDegrees());
        currentAngleDisplay.setDouble(getAngle().getDegrees());
        topSwitchDisplay.setBoolean(limitSwitchTop.get());
        bottomSwitchDisplay.setBoolean(limitSwitchBottom.get());

        if(!limitSwitchTop.get()) {
            speed1 = Math.min(0, speed1);
            speed2 = Math.min(0, speed2);
            resetEncoders(Rotation2d.fromDegrees(90));
        }

        if(limitSwitchBottom.get()) {
            speed1 = Math.max(0, speed1);
            speed2 = Math.max(0, speed2);
            resetEncoders(Rotation2d.fromDegrees(9));
        }

        raiseMotor1.set(speed1);
        raiseMotor2.set(speed2);
    }

    private double calculateFeedForward(Rotation2d angle){
        return Math.cos(angle.getRadians()) * Constants.EndEffectorConstants.armLevelVoltage;
    }

    private void resetEncoders(Rotation2d angle){
        double rawAbsolutePosition = angle.getRotations()/(Constants.EndEffectorConstants.armGearRatio*Constants.EndEffectorConstants.sprocketRatio);
        raiseEncoder1.setPosition(rawAbsolutePosition);
        raiseEncoder2.setPosition(rawAbsolutePosition);
    }

    private double getRawPosition1(){
        return raiseEncoder2.getPosition();
    }

    private double getRawPosition2(){
        return raiseEncoder1.getPosition();
    }
    
    private double getRawPosition(){
        return (getRawPosition1() + getRawPosition2())/2;
    }

    private Rotation2d getAngle1(){
        return Rotation2d.fromRotations(getRawPosition1()*Constants.EndEffectorConstants.armGearRatio*Constants.EndEffectorConstants.sprocketRatio);
    }

    private Rotation2d getAngle2(){
        return Rotation2d.fromRotations(getRawPosition1()*Constants.EndEffectorConstants.armGearRatio*Constants.EndEffectorConstants.sprocketRatio);
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(getRawPosition()*Constants.EndEffectorConstants.armGearRatio*Constants.EndEffectorConstants.sprocketRatio);
    }

    public void stop(){
        raiseMotor1.stopMotor();
        raiseMotor2.stopMotor();
    }

    public void setIdle(){
        raiseMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        raiseMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setTargetPosition(Rotation2d angle) {
        positionControl = true;
        targetAngle = angle;
    }

     public void setSpeed(double speed) {
        positionControl = false;
        speed1 = speed<0 ? speed*0.15 : speed*0.25;
        speed2 = speed<0 ? speed*0.15 : speed*0.25;
    }
}


// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.*;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// public class Arm extends SubsystemBase{

//     private final CANSparkMax raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
//     private final CANSparkMax raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
//     private final RelativeEncoder raiseEncoder1 = raiseMotor1.getEncoder();
//     private final RelativeEncoder raiseEncoder2 = raiseMotor2.getEncoder();

//     private final SparkPIDController armController1 = raiseMotor1.getPIDController();
//     private final SparkPIDController armController2 = raiseMotor2.getPIDController();

//     public Arm(){
//         raiseMotor2.setInverted(true);
//     }

//     public void periodic(){
//         SmartDashboard.putNumber("Raise Position", getPosition());
//     }



//     private double getRaise1Position(){
//         return raiseEncoder1.getPosition();
//     }
    
//     private double getRaise2Position(){
//         return raiseEncoder2.getPosition();
//     }

//     public double getPosition(){
//         return (getRaise1Position() + getRaise2Position())/2;
//     }

//     public void stop(){
//         raiseMotor1.stopMotor();
//         raiseMotor2.stopMotor();
//     }

//     public void setIdle(){
//         System.out.println("Set idle");
//         raiseMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
//         raiseMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
//     }

//     public void setTargetPosition(double raiseGoalState) {
//         this.armController1.setReference(raiseGoalState, ControlType.kPosition);
//         this.armController2.setReference(raiseGoalState, ControlType.kPosition);
//     }

//      public void setSpeed(double speed) {
//         this.armController1.setReference(speed/2, ControlType.kVelocity);
//         this.armController2.setReference(speed/2, ControlType.kVelocity);
//     }
// }