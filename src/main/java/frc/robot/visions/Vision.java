package frc.robot.visions;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.MultiTargetPNPResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

/**
 * A class that does vision things.
 * @author Elijah Kelly
 * @version 0.1.0
 */
public class Vision {

    private AprilCamera[] cameras;

    /**
     * Constructs a new Vision object.
     */
    public Vision() {
        cameras = new AprilCamera[] {
            new AprilCamera("Cam2", -0.311, -0.311, 0, -Math.PI/12, Math.PI, 0),
            new AprilCamera("ArduCam", -0.311, 0.311, 0, -Math.PI/12, Math.PI, 0)
            //new AprilCamera("Cam2", 0, 0, 0, 0, 0, 0),
            //new AprilCamera("Cam8", 0, 0, 0, 0, 0, 0)
        };
    }

    /**
     * Uses a different method to find pose estimate based on PhotonVision results that should work with only one tag seen.
     * @return A list of field-relative poses of the robot according to each camera that sees AprilTags,
     * may have length 0 if no cameras see tags
     */
    public List<VisionPose> getAprilTagResultsPhotonPose() {
        List<VisionPose> poseReports = new ArrayList<VisionPose>();
        for (AprilCamera camera : cameras)
            poseReports.addAll(camera.getAprilTagResult());
        return poseReports;
    }

    /**
     * Returns an array of the most recent pose reports from each camera.
     * @return The most recent poses in order of camera ID.
     */
    public VisionPose[] getRecentResults() {
        VisionPose[] recentResults = new VisionPose[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            AprilCamera camera = cameras[i];
            recentResults[i] = camera.clearResultsCache();
        }
        return recentResults;
    }
}
