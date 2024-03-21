package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;

public class Launcher extends SubsystemBase{
    private final CANSparkMax leftMotor = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(18, MotorType.kBrushless);
    public static boolean isSpinning = false;
    public static boolean checkDirection = true;

    public Launcher(){
    }

    public void setSpeed(double speed) {
        leftMotor.set(-speed);
        rightMotor.set(-speed);
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

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import frc.robot.commands.*;

// public class Launcher extends SubsystemBase{
//     private final CANSparkMax bottomMotor = new CANSparkMax(17, MotorType.kBrushless);
//     private final CANSparkMax topMotor = new CANSparkMax(218, MotorType.kBrushless);
//     private final SparkPIDController bottomController = bottomMotor.getPIDController();
//     private final SparkPIDController topController = topMotor.getPIDController();
//     public static boolean isSpinning = false;
//     public static boolean checkDirection = true;

//     private ShuffleboardTab launcherTab = Shuffleboard.getTab("Launcher Control");

//     private GenericEntry KPChooser = launcherTab.addPersistent("launcher kp", 0).getEntry();
//     // private GenericEntry KIChooser = launcherTab.addPersistent("launcher ki", 0).getEntry();
//     // private GenericEntry KDChooser = launcherTab.addPersistent("launcher kd", 0).getEntry();
//     private GenericEntry targetSpeedChooser = launcherTab.add("Target Launch Speed", 0).getEntry();
//     private GenericEntry bottomSpeedDisplay = launcherTab.add("bottom speed", 0).getEntry();
//     private GenericEntry topSpeedDisplay = launcherTab.add("top speed", 0).getEntry();

//     public Launcher(){
//         bottomMotor.setInverted(true);
//         topMotor.setInverted(true);
//     }

//     public void setSpeed(double speed) {
//         bottomController.setReference(speed, ControlType.kVelocity);
//         topController.setReference(speed, ControlType.kVelocity);
//     }

//     public void stop(){
//         setSpeed(0);
//         LED.setState(0, false);
//         LED.setState(2, false);
//     }

//     public double getBottomVelocity(){
//         return bottomMotor.getEncoder().getVelocity();
//     }

//     public double getTopVelocity(){
//         return topMotor.getEncoder().getVelocity();
//     }

//     @Override
//     public void periodic(){
//         bottomSpeedDisplay.setDouble(getBottomVelocity());
//         topSpeedDisplay.setDouble(getTopVelocity());
//         bottomController.setP(KPChooser.getDouble(bottomController.getP()));
//         topController.setP(KPChooser.getDouble(topController.getP()));

//         setSpeed(targetSpeedChooser.getDouble(0));
//     }

// }



// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import frc.robot.commands.*;

// public class Launcher extends SubsystemBase{
//     private final CANSparkMax bottomMotor = new CANSparkMax(17, MotorType.kBrushless);
//     private final CANSparkMax topMotor = new CANSparkMax(218, MotorType.kBrushless);
//     private final PIDController bottomController = new PIDController(Constants.EndEffectorConstants.bottomLaunchKP, Constants.EndEffectorConstants.bottomLaunchKI, Constants.EndEffectorConstants.bottomLaunchKD);
//     private final PIDController topController = new PIDController(Constants.EndEffectorConstants.topLaunchKP, Constants.EndEffectorConstants.topLaunchKI, Constants.EndEffectorConstants.topLaunchKD);
//     public double targetSpeed = 0;
//     public static boolean isSpinning = false;
//     public static boolean checkDirection = true;
//     private double topSpeed = 0;
//      private double bottomSpeed = 0;
    
//     private ShuffleboardTab launcherTab = Shuffleboard.getTab("Launcher Control");

//     private GenericEntry KPChooser = launcherTab.addPersistent("launcher kp", 0).getEntry();
//     // private GenericEntry KIChooser = launcherTab.addPersistent("launcher ki", 0).getEntry();
//     // private GenericEntry KDChooser = launcherTab.addPersistent("launcher kd", 0).getEntry();
//     private GenericEntry targetSpeedChooser = launcherTab.add("Target Launch Speed", 0).getEntry();
//     private GenericEntry bottomSpeedDisplay = launcherTab.add("bottom speed", 0).getEntry();

//     private GenericEntry topVoltageDisplay = launcherTab.add("Top Voltage", 0).getEntry();
//     private GenericEntry bottomVoltageDisplay = launcherTab.add("Bottom Voltage", 0).getEntry();

//     private GenericEntry topSpeedDisplay = launcherTab.add("top speed", 0).getEntry();

//     public Launcher(){
//         bottomMotor.setInverted(true);
//         topMotor.setInverted(true);
//     }

//     public void setSpeed(double speed) {
//         targetSpeed = speed;
//     }

//     public void stop(){
//         setSpeed(0);
//         LED.setState(0, false);
//         LED.setState(2, false);
//     }

//     public double getBottomVelocity(){
//         return bottomMotor.getEncoder().getVelocity();
//     }

//     public double getTopVelocity(){
//         return topMotor.getEncoder().getVelocity();
//     }

//     @Override
//     public void periodic(){
//         bottomController.setP(KPChooser.getDouble(bottomController.getP()));
//         topController.setP(KPChooser.getDouble(topController.getP()));
        
//         bottomSpeed = bottomSpeed*0.9 - bottomController.calculate(getBottomVelocity(), targetSpeed)/10;
//         topSpeed = topSpeed*0.9 - topController.calculate(getTopVelocity(), targetSpeed)/10;

//         bottomSpeedDisplay.setDouble(getBottomVelocity());
//         topSpeedDisplay.setDouble(getTopVelocity());

//         bottomVoltageDisplay.setDouble(bottomSpeed);
//         topVoltageDisplay.setDouble(topSpeed);

//         bottomMotor.set(bottomSpeed);
//         topMotor.set(topSpeed);
//         setSpeed(targetSpeedChooser.getDouble(0));
//     }

// }