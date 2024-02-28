package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {

  private final DoubleSolenoid extender;

  public Extender() {
    extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 3);
  }

  public void forward(){
    extender.set(DoubleSolenoid.Value.kForward);
    
  }
  public void backward(){
    extender.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
  }
}