package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase{

    private final CANSparkMax raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private final RelativeEncoder raiseEncoder1 = raiseMotor1.getEncoder();
    private final RelativeEncoder raiseEncoder2 = raiseMotor2.getEncoder();

    public Arm(){
        raiseMotor2.setInverted(true);
    }

    public void periodic(){
        SmartDashboard.putNumber("Raise Position", getRaise1Position());
    }

    public double getRaise1Position(){
        return raiseEncoder1.getPosition();
    }
    
    public double getRaise2Position(){
        return raiseEncoder2.getPosition();
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

    public void setSpeed(double speed) {
        raiseMotor1.set(speed);
        raiseMotor2.set(speed);
    }
}