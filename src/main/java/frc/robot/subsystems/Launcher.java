package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase{
    private final CANSparkMax left = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax right = new CANSparkMax(18, MotorType.kBrushless);
    public static boolean isSpinning = false;
    public static boolean checkDirection = true;

    public Launcher(){
    }

    public void intake() {
        left.set(0.1);
        right.set(-0.1);
    }

    public void outtake() {
        left.set(-0.1);
        right.set(0.1);
    }

    public void stop(){
        left.set(0);
        right.set(0);
    }

}