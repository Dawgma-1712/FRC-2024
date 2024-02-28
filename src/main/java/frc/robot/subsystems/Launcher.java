package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;

public class Launcher extends SubsystemBase{
    private final CANSparkMax left = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax right = new CANSparkMax(18, MotorType.kBrushless);
    private final CANSparkMax feed = new CANSparkMax(16, MotorType.kBrushless);
    public static boolean isSpinning = false;
    public static boolean checkDirection = true;

    public Launcher(){
        
    }

    public void outtake(){
        left.set(-0.1);
        right.set(0.1);
        feed.set(0.1);
        LED.setState(0, true);
    }

    public void stop(){
        left.set(0);
        right.set(0);
        LED.setState(0, false);
        LED.setState(2, false);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Launcher Speed", left.getEncoder().getVelocity());
        SmartDashboard.putNumber("Right Launcher Speed", right.getEncoder().getVelocity());
    }

}