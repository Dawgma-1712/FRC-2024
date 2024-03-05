package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LED;
import frc.robot.subsystems.*;

public class BeamBreak extends SubsystemBase{
    private DigitalInput input = new DigitalInput(3);

    public BeamBreak() {
    }

    public void periodic() {
        SmartDashboard.putBoolean("Note Obtained", input.get());
        LED.setState(6, input.get());
    }

    public boolean beamBreak() {
        return input.get();
    }
    
}
