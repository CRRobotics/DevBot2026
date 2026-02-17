package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/*
 * Based on 3015's 2024 robot code. Thanks much to their team for consultation during the 2025 FLR match!
 * Use three PID controllers (X, Y, rotation) to drive to a pose on the field, end velocity 0.
 *
 */
public class driveToRelative extends Command{
    protected final DriveSubsystem driveTrain;
    protected Supplier<Pose2d> targetPose;

    protected final ProfiledPIDController xController;
    protected final ProfiledPIDController yController;
    protected final ProfiledPIDController rotationController;

    public driveToRelative(DriveSubsystem dt) {
        this.driveTrain = dt;
        this.xController =
        new ProfiledPIDController(
            Constants.DriveConstants.autoMoveK.kP,
            Constants.DriveConstants.autoMoveK.kI,
            Constants.DriveConstants.autoMoveK.kD,
            Constants.DriveConstants.autoMoveConstraints);
    this.yController =
        new ProfiledPIDController(
            Constants.DriveConstants.autoMoveK.kP,
            Constants.DriveConstants.autoMoveK.kI,
            Constants.DriveConstants.autoMoveK.kD,
            Constants.DriveConstants.autoMoveConstraints);
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

    @Override  
    public void execute() {
        if (targetPose == null) return;

        Pose2d tp = targetPose.get();
        Pose2d cp = driveTrain.getPose();

        double xFeedback = xController.calculate(cp.getX(), tp.getX());
        double yFeedback = yController.calculate(cp.getY(), tp.getY());
        double rotFeedback = rotationController.calculate(
                cp.getRotation().getRadians(), tp.getRotation().getRadians());

        double xFF = xController.getSetpoint().velocity;
        double yFF = yController.getSetpoint().velocity;
        double rotFF = rotationController.getSetpoint().velocity;

        double xVel = xFF + xFeedback;
        double yVel = yFF + yFeedback;
        double rotVel = rotFF + rotFeedback;
 
        if (Math.abs(cp.getX() - tp.getX()) < Constants.DriveConstants.TOL_TRANSLATION) {
            xVel = 0;
        }
        if (Math.abs(cp.getY() - tp.getY()) < Constants.DriveConstants.TOL_TRANSLATION) {
            yVel = 0;
        }
        if (Math.abs(cp.getRotation().minus(tp.getRotation()).getDegrees()) < Constants.DriveConstants.TOL_ROTATION) {
            rotVel = 0;
        }

        // DataLogManager.log(String.format("AA:     X        Y        Theta"));
        // DataLogManager.log(
        //     String.format(
        //         " err: %4f %4f %4f",
        //         cp.getX() - tp.getX(),
        //         cp.getY() - tp.getY(),
        //         cp.getRotation().getDegrees() - tp.getRotation().getDegrees()));
        // DataLogManager.log(String.format("calc: %4f %4f %4f", xFeedback, yFeedback, rotFeedback));
        // DataLogManager.log(String.format("  ff: %4f %4f %4f", xFF, yFF, rotFF));
        // DataLogManager.log(String.format(" new: %4f %4f %4f", xVel, yVel, rotVel));

        driveTrain.drive(xVel, yVel, rotVel,true);
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) return true;

        Pose2d tp = targetPose.get();
        Pose2d cp = driveTrain.getPose();

        // Curious why 3015 didn't use a cartesian distance here?
        return Math.abs(cp.getX() - tp.getX()) < Constants.DriveConstants.TOL_TRANSLATION
                && Math.abs(cp.getY() - tp.getY()) < Constants.DriveConstants.TOL_TRANSLATION
                && Math.abs(cp.getRotation().minus(tp.getRotation())
                        .getDegrees()) < Constants.DriveConstants.TOL_ROTATION;
    }
}