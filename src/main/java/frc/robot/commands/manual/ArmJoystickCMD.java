package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ArmJoystickCMD extends Command{
    private Arm arm;
    private Supplier<Double> raiseValue;

    public ArmJoystickCMD(Arm arm, Supplier<Double> raiseValue) {
        this.arm = arm;
        this.raiseValue = raiseValue;
        addRequirements(arm);
    }

    public void initialize() {}

    @Override
    public void execute() {
        if(Math.abs(raiseValue.get()) > Constants.OperatorConstants.ArmDeadband){
            double raise = Math.abs(raiseValue.get()) > Constants.OperatorConstants.ArmDeadband ? raiseValue.get() : 0;
            arm.setSpeed(raise+ Constants.OperatorConstants.gOffset);
        }else{
            arm.setSpeed(0);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
    return false;
    }
}