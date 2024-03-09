package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;

public class Launcher extends SubsystemBase{
    private final CANSparkMax leftMotor = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(18, MotorType.kBrushless);
    // private final SparkPIDController leftController = leftMotor.getPIDController();
    // private final SparkPIDController rightController = rightMotor.getPIDController();
    public static boolean isSpinning = false;
    public static boolean checkDirection = true;

    public Launcher(){
    }

    public void setSpeed(double speed) {
        leftMotor.set(-speed);
        rightMotor.set(-speed);
        // leftController.setReference(-speed, ControlType.kVelocity);
        // rightController.setReference(-speed, ControlType.kVelocity);
    }

    public void stop(){
        setSpeed(0);
        LED.setState(0, false);
        LED.setState(2, false);
    }

    public double getRightVelocity(){
        return rightMotor.getEncoder().getVelocity();
    }

    public double getLeftVelocity(){
        return leftMotor.getEncoder().getVelocity();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Launcher Speed", leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Right Launcher Speed", rightMotor.getEncoder().getVelocity());
    }

}