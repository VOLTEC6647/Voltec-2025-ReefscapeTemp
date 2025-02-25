package com.team1678.frc2024.subsystems.vision;

import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.Constants1678.PoseEstimatorConstants;
import com.team1678.frc2024.RobotState.VisionUpdate;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.Util;
import com.team1678.lib.logger.LogUtil;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team6647.frc2025.FieldLayout;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;


public class VisionDevice extends Subsystem {
    private final VisionDeviceConstants mConstants;
    private PhotonCamera camera;
    private PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs = VecBuilder.fill(4, 4, 8); // Initialize with some default values
    private boolean is_connected = false;

    public VisionDevice(VisionDeviceConstants constants) {
        mConstants = constants;
        camera = new PhotonCamera(mConstants.kTableName);

        photonEstimator = new PhotonPoseEstimator(
                FieldLayout.kTagMap, //  AprilTagFieldLayout
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(new Translation3d(mConstants.kRobotToCamera.getX(),mConstants.kRobotToCamera.getY(),6.071),new Rotation3d(0,0,0))
				
        );
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }

    private void updateConnectivity() {
        // PhotonVision doesn't have a direct "is connected" method like the old NetworkTables approach.
        // We can approximate it by checking if we've received any results recently.
        var result = camera.getLatestResult();
        is_connected = result.hasTargets(); // Or some other reasonable check based on the result.
    }


   /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.  This is
     * very similar to the PhotonVision example, but adapted for our structure.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

            SmartDashboard.putString("Vision " + mConstants.kTableName + "/result2", estimatedPose.toString());
                

        if (estimatedPose.isEmpty()) {
            // No pose input. Default to high std devs, effectively ignoring the measurement.
            curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = VecBuilder.fill(4, 4, 8); // Base StdDevs, adjust as needed.
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            SmartDashboard.putBoolean("Vision " + mConstants.kTableName + "/test2", true);

            var tagPose = FieldLayout.kTagMap.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No tags visible.  High std devs, ignore measurement
            curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 0.5); // Reduce further - tune!
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4)
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30)); // Adjust scaling factor (30)
            curStdDevs = estStdDevs;
        }
    }


    @Override
    public void readPeriodicInputs() {
        updateConnectivity(); // Update connection status

        var result = camera.getLatestResult();
        SmartDashboard.putString("Vision " + mConstants.kTableName + "/result", result.toString());

        if (!result.hasTargets()) {
          return;
        }
        SmartDashboard.putString("Vision " + mConstants.kTableName + "/result3", result.toString());


        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }

        if (visionEst.isPresent()) {
            EstimatedRobotPose estimatedPose = visionEst.get();
            Pose2d robotPose = Pose2d.fromLegacy(estimatedPose.estimatedPose.toPose2d());
            double timestamp = estimatedPose.timestampSeconds;

            // Add vision update to RobotState
            // Adjust standard deviations as needed
            double xyStdDev = curStdDevs.get(0, 0); // Use calculated std dev

             if (VisionDeviceManager.visionDisabled()) {
				return;
			}

            SmartDashboard.putNumber("Vision " + mConstants.kTableName + "/device/timestamp", timestamp);
            SmartDashboard.putString("Vision " + mConstants.kTableName + "/device/robotPose", robotPose.getTranslation().toString());
            SmartDashboard.putString("Vision " + mConstants.kTableName + "/device/kRobotToCamera", mConstants.kRobotToCamera.getTranslation().toString());
            SmartDashboard.putNumber("Vision " + mConstants.kTableName + "/device/xyStdDev", xyStdDev);



            RobotState.getInstance().addVisionUpdate(
                    new VisionUpdate(
                            timestamp,
                            robotPose.getTranslation(),
                            mConstants.kRobotToCamera.getTranslation(), // Use the correct camera offset
                            xyStdDev
                    )
            );

            // Calculate and log vision heading
            double rotation_degrees = robotPose.getRotation().getDegrees() + 180.0; // Same logic as before
             if (!com.team1678.frc2024.Robot1678.is_red_alliance) { // Replace with your alliance check.
                rotation_degrees = Util.boundAngleNeg180to180Degrees(rotation_degrees);
            } else {
                rotation_degrees = Util.boundAngle0to360Degrees(rotation_degrees);
            }
            SmartDashboard.putNumber("Vision Heading/" + mConstants.kTableName, rotation_degrees);
			VisionDeviceManager.getInstance().getMovingAverage().addNumber(rotation_degrees);

            //Logging
            LogUtil.recordPose2d("Vision " + mConstants.kTableName + "/Robot Pose", robotPose);
            LogUtil.recordPose2d(
					"Vision " + mConstants.kTableName + "/Relevant Odometry Pose",
					RobotState.getInstance().getFieldToVehicle(timestamp));
        }
    }


    @Override
    public void outputTelemetry() {
       // Logger.recordOutput("Vision " + mConstants.kTableName + "/Last Update Timestamp Timestamp", mPeriodicIO.latest_timestamp); // No longer needed
        //Logger.recordOutput("Vision " + mConstants.kTableName + "/N Queued Updates", mPeriodicIO.frames.size()); // No longer needed
        SmartDashboard.putBoolean("Vision " + mConstants.kTableName + "/is Connnected", is_connected);
        SmartDashboard.putNumber("Vision/" + mConstants.kTableName + " Std Dev", curStdDevs.get(0,0));

    }

    @Override
    public void writePeriodicOutputs() {
        // No-op
    }
    
    public boolean isConnected() {
        return is_connected;
    }

      public void captureCalibrationFrame() {
        // Not directly supported by PhotonVision in the same way.  Calibration is usually handled
        // through the PhotonVision UI. If you have a specific calibration routine, you'd trigger
        // it differently (e.g., a button press that sets a flag, which you then check here).
    }
}