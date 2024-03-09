package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase{

    private final CANSparkMax raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private final RelativeEncoder raiseEncoder1 = raiseMotor1.getEncoder();
    private final RelativeEncoder raiseEncoder2 = raiseMotor2.getEncoder();

    private final SparkPIDController armController1 = raiseMotor1.getPIDController();
    private final SparkPIDController armController2 = raiseMotor2.getPIDController();

    public Arm(){
        raiseMotor2.setInverted(true);
    }

    public void periodic(){
        SmartDashboard.putNumber("Raise Position", getPosition());
    }



    private double getRaise1Position(){
        return raiseEncoder1.getPosition();
    }
    
    private double getRaise2Position(){
        return raiseEncoder2.getPosition();
    }

    public double getPosition(){
        return (getRaise1Position() + getRaise2Position())/2;
    }

    public void stop(){
        raiseMotor1.stopMotor();
        raiseMotor2.stopMotor();
    }

    public void setIdle(){
        System.out.println("Set idle");
        raiseMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        raiseMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setTargetPosition(double raiseGoalState) {
        this.armController1.setReference(raiseGoalState, ControlType.kPosition);
        this.armController2.setReference(raiseGoalState, ControlType.kPosition);
    }

     public void setSpeed(double speed) {
        this.armController1.setReference(speed/2, ControlType.kVelocity);
        this.armController2.setReference(speed/2, ControlType.kVelocity);
    }
}