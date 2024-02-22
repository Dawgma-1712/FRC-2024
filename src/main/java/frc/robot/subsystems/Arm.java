package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase{

    private final CANSparkMax raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax extendMotor = new CANSparkMax(15, MotorType.kBrushless);

    private final RelativeEncoder raiseEncoder1 = raiseMotor1.getEncoder();
    private final RelativeEncoder raiseEncoder2 = raiseMotor2.getEncoder();
    private final RelativeEncoder extendEncoder = extendMotor.getEncoder();

    private boolean isCone = false;//Change if initial mode is different

    private final PIDController armExtendPID = new PIDController(0.27891030029999945400000000000, 0, 0);
    private final PIDController armRaisePID1 = new PIDController(0.028600000000000, 0.005, 0.008);
    private final PIDController armRaisePID2 = new PIDController(0.028600000000000, 0.005, 0.008);

    private final Spark LED = new Spark(0);

    private double extendGoalState = 0.0, raiseGoalState = 0.0;
    private String currentState = "";

    public Arm(){
        raiseMotor2.setInverted(true);
    }

    public void periodic(){
        new Thread(() -> {
            extendMotor.set(armExtendPID.calculate(getExtendPosition(), extendGoalState));
        }).start();
        new Thread(() -> {
            raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), raiseGoalState));
        }).start();
        new Thread(() -> {
            raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), raiseGoalState));
        }).start();
        SmartDashboard.putNumber("Extend Goal Position", extendGoalState);
        SmartDashboard.putNumber("Raise Goal Position", raiseGoalState);
        SmartDashboard.putNumber("Extend Position", getExtendPosition());
        SmartDashboard.putNumber("Raise Position", getRaise1Position());
        SmartDashboard.putString("Stage", currentState);
        SmartDashboard.putBoolean("Cone Mode?", isCone);
    }

    public double getRaise1Position(){
        return raiseEncoder1.getPosition();
    }
    public double getRaise2Position(){
        return raiseEncoder2.getPosition();
    }
    public double getExtendPosition(){
        return extendEncoder.getPosition();
    }
    public void setIsCone(boolean isCone){
        this.isCone = isCone;
    }
    public boolean getIsCone(){
        return isCone;
    }
    public void togleMode(){
        isCone = !isCone;
        if(isCone){
            LED.set(0.69);
        }
        if(!isCone){
            LED.set(0.91);
        }
    }
    public void stop(){
        raiseMotor1.stopMotor();
        raiseMotor2.stopMotor();
        extendMotor.stopMotor();
    }

    public void setIdle(){
        System.out.println("Set idle");
        extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        raiseMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        raiseMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setPreset(String stage){
        if(isCone && (stage.equals("cubeMid") || stage.equals("cubeHigh"))){
            if(stage.equals("cubeMid")){
                stage = "coneMid";
            }
            if(stage.equals("cubeHigh")){
                stage = "coneHigh";
            }
        }
        currentState = stage;
        extendGoalState = OperatorConstants.armExtendPresets.get(stage);
        raiseGoalState = OperatorConstants.armRaisePresets.get(stage);

        /*
        new Thread(() -> {
            //armExtendPID.setReference(OperatorConstants.armExtendPresets.get(stage), ControlType.kPosition);
            extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get(stage)));
        }).start();
        new Thread(() -> {
            //armRaisePID1.setReference(OperatorConstants.armRaisePresets.get(stage), ControlType.kPosition);
            raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get(stage)));
        }).start();
        new Thread(() -> {
            //armRaisePID2.setReference(OperatorConstants.armRaisePresets.get(stage), ControlType.kPosition);
            raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), OperatorConstants.armRaisePresets.get(stage)));
        }).start();
        */
    }

    public void manualArm2(double extend, double raise){
        //extendMotor.set(armExtendPIDM.calculate(getExtendPosition(), getExtendPosition() + 1.5*extend));
        //raiseMotor1.set(armRaisePID1M.calculate(getRaise1Position(), getRaise1Position() + raise));
        //raiseMotor2.set(armRaisePID2M.calculate(getRaise2Position(), getRaise2Position() + raise));
        /*
        if(raise > 0){
        raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), getRaise1Position() + raise) * 1.5);
        raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), getRaise2Position() + raise) * 1.5);
        }
        else{
            raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), getRaise1Position() + raise));
            raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), getRaise2Position() + raise));
        }
        */
    }

    public void manualArm(double extend, double raise) {
        //extendMotor.set(extend);
        //raiseMotor1.set(raise/10);
        //raiseMotor2.set(raise/10);
        extendGoalState += extend;
        raiseGoalState += raise/5.0;
    }
}