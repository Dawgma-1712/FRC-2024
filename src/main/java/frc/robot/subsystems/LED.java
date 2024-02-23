package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED {
    private final Spark LED = new Spark(0);

    enum ColorState {
        Aligning,
        Aligned,
        SwerveLocked,
        SlowMode,
        Intaking,
        NoteObtained,
        Launching,
        Idle
    }
}
