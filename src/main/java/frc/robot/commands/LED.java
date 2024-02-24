package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;

public class LED extends SubsystemBase{
    private final  AddressableLED LED = new AddressableLED(0);
    private int[] rgb = new int[4]; //rbg values, blinking is 1
    private int which = 0;

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

    public int[] currentRGB(){
        ColorState state = ColorState.values()[which];
        switch (state){
            case Aligning:
                rgb[0] = 0;
                rgb[1] = 255;
                rgb[2] = 0;
                rgb[3] = 1; //blinking
                break;
            case Aligned:
                rgb[0] = 0;
                rgb[1] = 255;
                rgb[2] = 0;
                rgb[3] = 0; //not blinking
                break;
            case SwerveLocked:
                rgb[0] = 255;
                rgb[1] = 0;
                rgb[2] = 0;
                rgb[3] = 0;
                break;
            case SlowMode:
                rgb[0] = 255;
                rgb[1] = 255;
                rgb[2] = 0;
                rgb[3] = 0;
                break;
            case Intaking:
                rgb[0] = 255;
                rgb[1] = 140;
                rgb[2] = 0;
                rgb[3] = 1;
                break;
            case NoteObtained:
                rgb[0] = 255;
                rgb[1] = 140;
                rgb[2] = 0;
                rgb[3] = 0;
                break;
            case Launching:  
                rgb[0] = 0;
                rgb[1] = 0;
                rgb[2] = 0;
                rgb[3] = 2; //rainbow
                break;
            case Idle:
                rgb[0] = 255;
                rgb[1] = 20;
                rgb[2] = 147;
                rgb[3] = 0;
                break;
        }
    
        return(rgb);
    }
}
