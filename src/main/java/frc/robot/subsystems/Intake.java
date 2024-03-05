package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;

public class Intake extends SubsystemBase{
    private final CANSparkMax intake = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax feed = new CANSparkMax(16, MotorType.kBrushless);

    public Intake(){
    }

    public void intake() {
        intake.set(0.1);
        feed.set(0.1);
        LED.setState(2, true);
    }

    public void stop(){
        intake.set(0);
        feed.set(0);
        LED.setState(0, false);
        LED.setState(2, false);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Speed", intake.getEncoder().getVelocity());
    }

}