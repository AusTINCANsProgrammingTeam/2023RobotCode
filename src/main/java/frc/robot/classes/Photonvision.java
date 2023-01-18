package frc.robot.classes;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Photonvision {
    private static final AprilTagFieldLayout aprilTags = new AprilTagFieldLayout(
        List.of(
            new AprilTag(1, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(1))),
            new AprilTag(2, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(2))),
            new AprilTag(3, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(3))),
            new AprilTag(4, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(4))),
            new AprilTag(5, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(5))),
            new AprilTag(6, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(6))),
            new AprilTag(7, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(7))),
            new AprilTag(8, FieldConstants.aprilTagFlip(FieldConstants.aprilTags.get(8)))
        ),
        FieldConstants.fieldLength,
        FieldConstants.fieldWidth
    );

    private static final double cameraX = Units.inchesToMeters(0); //Distance from centerline of robot
    private static final double cameraY = Units.inchesToMeters(0); //Distance from center of robot
    private static final double cameraZ = Units.inchesToMeters(0); //Height from ground
    private static final double cameraRY = Units.degreesToRadians(0); //Mounting bracket angle
    private static final PhotonCamera camera = new PhotonCamera("OV5647");
    private static final Transform3d cameraTransform = new Transform3d(
        new Pose3d(), 
        new Pose3d(
            new Translation3d(
                cameraX,
                cameraY,
                cameraZ
            ),
            new Rotation3d(
                0,
                cameraRY, 
                0)
        )
    );

    private static RobotPoseEstimator poseEstimator = new RobotPoseEstimator(
        aprilTags, 
        PoseStrategy.AVERAGE_BEST_TARGETS, 
        List.of(new Pair<PhotonCamera, Transform3d>(camera, cameraTransform))
    );

    public static Optional<Pair<Pose3d,Double>> getPoseUpdate(){
        return poseEstimator.update();
    }

    public static Integer getAprilTagID(){
        PhotonPipelineResult result = camera.getLatestResult();
        if(result.hasTargets()){
            return result.getBestTarget().getFiducialId();
        }
        return -1;
    }
}  
