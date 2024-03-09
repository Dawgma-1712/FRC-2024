package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.*;

public class Feed extends SubsystemBase{
    private final CANSparkMax feedMotor = new CANSparkMax(16, MotorType.kBrushless);
    private final SparkPIDController feedController = feedMotor.getPIDController();

    public Feed(){
    }

    public void setSpeed(double speed) {
        if(Math.abs(speed)>Constants.EndEffectorConstants.feedDeadband){
            feedController.setReference(speed, ControlType.kVelocity);
        } else{
            feedController.setReference(0, ControlType.kVelocity);
        }
    }

    public void feed(){
        setSpeed(0.1);
        LED.setState(0, true);
    }

    public void stop(){
        setSpeed(0);
        LED.setState(0, false);
        LED.setState(2, false);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Feed Motor Speed", feedMotor.getEncoder().getVelocity());
    }

}