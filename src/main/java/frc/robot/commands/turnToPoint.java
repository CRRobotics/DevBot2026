package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/*
 * Based on 3015's 2024 robot code. Thanks much to their team for consultation during the 2025 FLR match!
 * Use three PID controllers (X, Y, rotation) to drive to a pose on the field, end velocity 0.
 *
 */
public class turnToPoint extends Command{
    protected final DriveSubsystem driveTrain;
    private boolean IsRed;
    protected final ProfiledPIDController rotationController;
    private double angle;

    public turnToPoint(DriveSubsystem dt) {
        this.driveTrain = dt;
        this.rotationController =
        new ProfiledPIDController(
            Constants.DriveConstants.autoTurnK.kP,
            Constants.DriveConstants.autoTurnK.kI,
            Constants.DriveConstants.autoTurnK.kD,
            Constants.DriveConstants.autoTurnConstraints);
    this.rotationController.setIZone(Constants.DriveConstants.autoTurnK.iZone);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.driveTrain);
    }

    private void CheckAlliance(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red){
                IsRed = true;
            }
            else {
                IsRed = false;
            }
        }
        else {
            IsRed= false;
        };
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {

        Pose2d cp = driveTrain.getPose();
        CheckAlliance();
        // angle in degrees
        if (IsRed == true){
            angle = Math.atan((FieldConstants.redhubY - cp.getY())/(FieldConstants.redhubX - cp.getX()));
        }
        else {
            angle = Math.atan((FieldConstants.bluehubY - cp.getY())/(FieldConstants.bluehubX - cp.getX()));

        }


        double rotFeedback = rotationController.calculate(
                cp.getRotation().getRadians(),angle + Math.PI);

        double rotFF = rotationController.getSetpoint().velocity;

        double rotVel = rotFF + rotFeedback;

        if (Math.abs(cp.getRotation().getDegrees() - (angle  * (180/Math.PI)) + 180) < Constants.DriveConstants.TOL_ROTATION) {
            rotVel = 0;
        }

        System.out.println(rotVel);
        driveTrain.drive(0, 0, rotVel,true);
    }

    @Override
    public boolean isFinished() {
        Pose2d cp = driveTrain.getPose();

        return Math.abs(cp.getRotation().getDegrees() - (angle  * (180/Math.PI) + 180)) < Constants.DriveConstants.TOL_ROTATION;
    }
}