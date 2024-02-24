package frc.robot.commands;

import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class IntakeCMD extends Command{
    private boolean previous;    
    Intake intake;
    BeamBreak sensor;

    public IntakeCMD(Intake intake, BeamBreak sensor){
        this.intake = intake;
        this.sensor = sensor;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
     
    }


    @Override
    public void execute(){
        intake.intake();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return sensor.beamBreak();
    }


}

    