package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class printCommand extends Command{
    public printCommand(){
        //SmartDashboard.putBoolean("RanTestCommand", false);
        addRequirements();
    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean("RanTestCommand", false);
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("RanTestCommand", true);
        System.out.println("ran printcommand");
    }

    @Override 
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("RanTestCommand", false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
