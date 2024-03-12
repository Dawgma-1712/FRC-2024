package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private DigitalInput limitSwitchTop = new DigitalInput(0);
    private DigitalInput limitSwitchBottom = new DigitalInput(1);

    private final PIDController armRaisePID1 = new PIDController(Constants.EndEffectorConstants.armRaiseKP, Constants.EndEffectorConstants.armRaiseKI, Constants.EndEffectorConstants.armRaiseKD);
    private final PIDController armRaisePID2 = new PIDController(Constants.EndEffectorConstants.armRaiseKP, Constants.EndEffectorConstants.armRaiseKI, Constants.EndEffectorConstants.armRaiseKD);

    private double raiseGoalState = 0.0;

    private boolean positionControl = false;

    public Arm(){
        raiseMotor2.setInverted(true);
        System.out.println("Arm instantiated!");
    }

    public void periodic(){
        if(positionControl){
            raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), raiseGoalState));
            raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), raiseGoalState));
        }
        
        SmartDashboard.putNumber("Raise Goal Position", raiseGoalState);
        SmartDashboard.putNumber("Raise Position", getPosition());

        if(limitSwitchTop.get()) {
            raiseEncoder1.setPosition(Constants.OperatorConstants.topSwitch);
            raiseEncoder2.setPosition(Constants.OperatorConstants.topSwitch);

            if(raiseEncoder1.getVelocity() < 0) {
                stop();
            }
        }

        if(limitSwitchBottom.get()) {
            raiseEncoder1.setPosition(Constants.OperatorConstants.bottomSwitch);
            raiseEncoder2.setPosition(Constants.OperatorConstants.bottomSwitch);

            if(raiseEncoder1.getVelocity() > 0) {
                stop();
            }
        }
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
        positionControl = true;
        this.raiseGoalState = raiseGoalState;
    }

     public void setSpeed(double speed) {
        positionControl = false;
        // if(speed<0){
            raiseMotor1.set(-speed*0.15);
            raiseMotor2.set(-speed*0.15);
        // }else{
        //     raiseMotor1.set(0);
        //     raiseMotor2.set(0);
        // }

    }
}

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.*;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// public class Arm extends SubsystemBase{

//     private final CANSparkMax raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
//     private final CANSparkMax raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
//     private final RelativeEncoder raiseEncoder1 = raiseMotor1.getEncoder();
//     private final RelativeEncoder raiseEncoder2 = raiseMotor2.getEncoder();

//     private final SparkPIDController armController1 = raiseMotor1.getPIDController();
//     private final SparkPIDController armController2 = raiseMotor2.getPIDController();

//     public Arm(){
//         raiseMotor2.setInverted(true);
//     }

//     public void periodic(){
//         SmartDashboard.putNumber("Raise Position", getPosition());
//     }



//     private double getRaise1Position(){
//         return raiseEncoder1.getPosition();
//     }
    
//     private double getRaise2Position(){
//         return raiseEncoder2.getPosition();
//     }

//     public double getPosition(){
//         return (getRaise1Position() + getRaise2Position())/2;
//     }

//     public void stop(){
//         raiseMotor1.stopMotor();
//         raiseMotor2.stopMotor();
//     }

//     public void setIdle(){
//         System.out.println("Set idle");
//         raiseMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
//         raiseMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
//     }

//     public void setTargetPosition(double raiseGoalState) {
//         this.armController1.setReference(raiseGoalState, ControlType.kPosition);
//         this.armController2.setReference(raiseGoalState, ControlType.kPosition);
//     }

//      public void setSpeed(double speed) {
//         this.armController1.setReference(speed/2, ControlType.kVelocity);
//         this.armController2.setReference(speed/2, ControlType.kVelocity);
//     }
// }