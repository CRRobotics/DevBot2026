package frc.robot.visions;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.visions.VisionPose;

/**
 * A class meant to store the relevant data and objects to an AprilTag detection camera.
 * @author Elijah Kelly
 * @version 0.0.0
 */
public class AprilCamera {
    private String name;
    private PhotonCamera photonCamera;
    private Transform3d robotToCam;
    private PhotonPoseEstimator photonPoseEstimator;
    private VisionPose resultsCache;

    /**
     * Constructs a new AprilCamera object with the given data.
     * @param name The camera name in PhotonVision
     * @param x The robot-relative x coordinate
     * @param y The robot-relative y coordinate
     * @param z The robot-relative z coordinate
     * @param pitch The robot-relative pitch
     * @param yaw The robot-relative yaw
     * @param roll The robot-relative roll
     */
    public AprilCamera(String name, double x, double y, double z, double pitch, double yaw, double roll) {
        this.name = name;
        this.photonCamera = new PhotonCamera(name);
        this.robotToCam = new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw)); // Pitch, yaw, and roll's order may need to change
        
        //AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); //Default apriltag field


        AprilTagFieldLayout customLayout = null;
        File theFile = new File(Filesystem.getDeployDirectory(), "AprilTagCalibration.json");
        
        try {
            String p = theFile.getAbsolutePath();
            customLayout = new AprilTagFieldLayout(theFile.getAbsolutePath()); //Calibration apriltag field
            System.out.print("Found " + p + "\n");
        } catch (IOException e) {
            System.out.print("Failed to load custom field layout" + e + "\n"); 
        }

        //AprilTagFieldLayout customLayout = AprilTagFieldLayout.loadFromResource("src/ApriltagCalibration.json");

        this.photonPoseEstimator = new PhotonPoseEstimator(customLayout, robotToCam);
        this.resultsCache = null;
    }

    /**
     * Uses PhotonVision data from camera to estimate the robot pose.
     * @return The estimated pose for each unread result
     */
    public List<VisionPose> getAprilTagResult() {
        List<VisionPose> poseReports = new ArrayList<VisionPose>();
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> estimatedPoseIfPresent = photonPoseEstimator.estimateCoprocMultiTagPose(result); // Probably happens on coprocessor
            if (estimatedPoseIfPresent.isEmpty()) // This will happen if it doesn't see multiple tags
                estimatedPoseIfPresent = photonPoseEstimator.estimateLowestAmbiguityPose(result); // Single-tag method

            if (estimatedPoseIfPresent.isPresent()) {
                Pose3d estimatedPose = estimatedPoseIfPresent.get().estimatedPose;
                double timestamp = result.getTimestampSeconds();
                VisionPose toReturn = new VisionPose(estimatedPose, timestamp);

                // Display stuff
                SmartDashboard.putNumber(name + " X", estimatedPose.getX());
                SmartDashboard.putNumber(name + " Y", estimatedPose.getY());
                SmartDashboard.putNumber(name + " Theta", Math.toDegrees(estimatedPose.getRotation().getZ()));

                // Not sure of the time-order of multiple results for the same camera; do we get the oldest
                // or the newest by just taking the last? Not a big issue, we are polling every 20ms
                resultsCache = toReturn;
                poseReports.add(toReturn);
            }
        }
        
        return poseReports;
    }

    /**
     * Returns the specified camera name. This should match the name in PhotonVision.
     * @return The camera's name
     */
    public String getName() {
        return name;
    }

    /**
     * Returns the pose of the camera relative to the robot.
     * @return A robot-to-camera transform
     */
    public Transform3d getRobotToCam() {
        return robotToCam.div(1);
    }

    /**
     * Returns the last read result.
     * @return The last calculated pose of the robot and its timestamp
     */
    public VisionPose getResultsCache() {
        return resultsCache;
    }

    /**
     * Sets this camera's results cache (accessible via getResultsCache) to null.
     * @return The value of the results cache before cleared
     */
    public VisionPose clearResultsCache() {
        VisionPose temp = resultsCache;
        resultsCache = null;
        return temp;
    }

    @Override
    /**
     * Returns a string representation of the camera.
     */
    public String toString() {
        return "VisionCamera: name = " + name
            + "robotToCam = " + robotToCam
            + "resultsCache = " + resultsCache;
    }

    @Override
    /**
     * Returns whether this object is equal to the specified otherObject.
     * @param otherObject The object to compare to
     * @return true if the objects are equal, false if not.
     */
    public boolean equals(Object otherObject) {
        if (otherObject == null || otherObject.getClass() != AprilCamera.class)
            return false;
        AprilCamera other = (AprilCamera)otherObject;

        return other.name.equals(this.name)
            && other.robotToCam.equals(this.robotToCam)
            && other.photonPoseEstimator.equals(this.photonPoseEstimator);
    }
}
