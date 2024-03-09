package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.*;

public class Intake extends SubsystemBase{
    private final CANSparkMax intakeMotor = new CANSparkMax(15, MotorType.kBrushless);
    private final SparkPIDController intakeController = intakeMotor.getPIDController();

    public Intake(){
    }

    public void setSpeed(double speed) {
        //LED.setState(0, true);
        if(speed > Constants.EndEffectorConstants.intakeDeadband){
             intakeController.setReference(speed, ControlType.kVelocity);
        }else{
             intakeController.setReference(0, ControlType.kVelocity);
        }
    }

    public void stop(){
        setSpeed(0);
        LED.setState(0, false);
        LED.setState(2, false);
    }

    public double getVelocity() {
        return intakeMotor.getEncoder().getVelocity();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Launcher Speed", intakeMotor.getEncoder().getVelocity());
    }

}