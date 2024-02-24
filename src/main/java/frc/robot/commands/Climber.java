package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class Climber extends Command{    
    Arm check;
    Extender climb;
    boolean isExtended = false;

    public Climber(Arm Check, Extender climb){
        this.check = check;
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
     
    }


    @Override
    public void execute(){
        if(!(isExtended) && (check.getRaise1Position() > -5 && check.getRaise1Position() < -5) && (check.getRaise2Position() < 5.0 && check.getRaise2Position() > -5.0)){
            climb.forward();
        }else if(isExtended){
            climb.backward();
        }else{
            System.out.println("Welp you messed something up");
        }
        isExtended = !isExtended;
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }


}

    