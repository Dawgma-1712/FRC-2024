package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final DoubleSolenoid extender;
  private boolean extended = false;

  public Climber() {
    extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 3);
  }


  public void forward(){
    extender.set(DoubleSolenoid.Value.kForward);
    extended = true;
    
  }

  public void backward(){
    extender.set(DoubleSolenoid.Value.kReverse);
    extended = false;
  }

  public void toggle(){
    if(extended){
      backward();
    }else{
      forward();
    }
  }

  @Override
  public void periodic() {
  }
}