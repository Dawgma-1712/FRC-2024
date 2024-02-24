package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.*;

public class LED extends Command{

    private int[] rgb = new int[4]; //rbg values, blinking is 
    private static boolean[] curStates = new boolean[7];
    private int which = 0;
    private int m_autonomousCommand;

    
    // enum ColorState {
    //     Aligning, 
    //     Aligned,
    //     SwerveLocked,
    //     SlowMode,
    //     Intaking,
    //     NoteObtained,
    //     Launching,
    //     Idle
    // }
    public int[] currentRGB(){
        if(curStates[0]){ //Launching (works?) - rainbow
            rgb[0] = 0;
            rgb[1] = 0;
            rgb[2] = 0;
            rgb[3] = 2; //rainbow
        }else if(curStates[1]){ //Swerve Locked (works?) - solid red
            rgb[0] = 255;
            rgb[1] = 0;
            rgb[2] = 0;
            rgb[3] = 0;
        }
        else if(curStates[2]){ //Intaking (works?) - blinking orange
            rgb[0] = 255;
            rgb[1] = 140;
            rgb[2] = 0;
            rgb[3] = 1;
        }else if(curStates[3]){ //Aligning (works?) - blinking green
            rgb[0] = 0;
            rgb[1] = 255;
            rgb[2] = 0;
            rgb[3] = 1; //blinking
        }else if(curStates[4]){ //Aligned - blinking green
            rgb[0] = 0;
            rgb[1] = 255;
            rgb[2] = 0;
            rgb[3] = 0; //not blinking
        }else if(curStates[5]){ //Slow Mode (works?) - solid blue
            rgb[0] = 0;
            rgb[1] = 0;
            rgb[2] = 255;
            rgb[3] = 0;
        }else if(curStates[6]){ //Note Obtained (works?) - solid orange
            rgb[0] = 255;
            rgb[1] = 140;
            rgb[2] = 0;
            rgb[3] = 0;
        }else{ //Idle *works by default - hot pink
            rgb[0] = 255;
            rgb[1] = 20;
            rgb[2] = 147;
            rgb[3] = 0;
        }

        /* 
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
        */
    
        return(rgb);
    }

    public static void setState(int idx, boolean state){
        curStates[idx] = state;
    }
}
