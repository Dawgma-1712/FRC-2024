// package frc.robot.commands.presets;

// import frc.robot.Constants;
// import frc.robot.subsystems.*;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;


// public class IntakeCMD extends Command{
//     Intake intake;
//     Arm arm;
//     Feed feed;

//     public IntakeCMD(Intake intake, Arm arm, Feed feed, BeamBreak beamBreak){
//         this.arm = arm;
//         this.intake = intake;
//         this.feed = feed;
//         addRequirements(arm, intake, feed);
//     }


//     @Override
//     public void execute(){
//         intake.setSpeed(Constants.EndEffectorConstants.intakeSpeed);
//         feed.setSpeed(Constants.EndEffectorConstants.feedSpeed);
//     }

//     @Override
//     public void end(boolean interrupted){
//         intake.setSpeed(0);
//         feed.setSpeed(0);
//     }

//     @Override
//     public boolean isFinished(){
//         return beamBreak.hasNote();
//     }
// }

    