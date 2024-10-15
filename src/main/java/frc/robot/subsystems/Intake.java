package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

public class Intake extends SubsystemBase{
    private final CANSparkMax intakeMotor = new CANSparkMax(15, MotorType.kBrushless);

    public Intake(){
    }

    public void setSpeed(double speed) {
        if(speed != 0) LED.setState(0, true);
        intakeMotor.set(speed);
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