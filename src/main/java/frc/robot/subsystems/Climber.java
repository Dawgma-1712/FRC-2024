package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final Compressor compressor = new Compressor(19, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid extender = new DoubleSolenoid(19, PneumaticsModuleType.REVPH, 1, 3);
  private boolean extended = false;

  public Climber() {
    compressor.enableAnalog(Constants.OperatorConstants.minPressure, Constants.OperatorConstants.maxPressure);
    System.out.println("Compressor enabled");
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
    extended = !extended;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Compressor Enabled?", compressor.isEnabled());
  }
}