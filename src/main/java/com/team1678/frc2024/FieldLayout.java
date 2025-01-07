package com.team1678.frc2024;

import com.team1678.frc2024.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.util.Vector;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	public static double kFieldLength = Units.inchesToMeters(651.223);
	public static double kFieldWidth = Units.inchesToMeters(323.277);
	public static double kWingX = Units.inchesToMeters(229.201);
	public static double kPodiumX = Units.inchesToMeters(126.75);
	public static double kStartingLineX = Units.inchesToMeters(74.111);

	public static final double kApriltagWidth = Units.inchesToMeters(6.50);
	public static final AprilTagFieldLayout kTagMap;

	static {
		try {
			kTagMap = new AprilTagFieldLayout("/edu/wpi/first/apriltag/2025-reefscape.json");
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	// center notes labeled 1-5, with 1 being closest to the fms table
	public static Translation2d kCenterNote5 = new Translation2d(kFieldLength / 2.0, 0.752856);
	public static Translation2d kCenterNote4 = new Translation2d(kFieldLength / 2.0, 2.429256);
	public static Translation2d kCenterNote3 = new Translation2d(kFieldLength / 2.0, 4.105656);
	public static Translation2d kCenterNote2 = new Translation2d(kFieldLength / 2.0, 5.782056);
	public static Translation2d kCenterNote1 = new Translation2d(kFieldLength / 2.0, 7.458456);

	public static Translation2d[] kCenterNotes =
			new Translation2d[] {kCenterNote1, kCenterNote2, kCenterNote3, kCenterNote4, kCenterNote5};

	public static Translation2d kAmpCenter =
			new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

	/** Center of the speaker opening (blue alliance) */
	public static Pose2d kSpeakerCenter = new Pose2d(0.2, kFieldWidth - Units.inchesToMeters(104.0), new Rotation2d());

	public static Pose2d kCoralCenter = new Pose2d((kTagMap.getTagPose(19).get().getX()+kTagMap.getTagPose(20).get().getX())/2,kTagMap.getTagPose(18).get().getY(),new Rotation2d());

	public static double kCoralDistance = 46.75; //Please Fix
	public static double kCoralDistanceOffset = 10.0f; // temproral

	// hacer
	public static final double topCoralAngle = 90.0;
	public static final double topLeftCoralAngle = 100.0;
	public static final double topRightCoralAngle = 100.0;
	public static final double bottomCoralAngle = 100.0;
	public static final double bottomLeftCoralAngle = 100.0;
	public static final double bottomRightCoralAngle = 100.0;

	public enum CoralTarget {
		TOP(3.0),
		TOP_LEFT(3.0),
		TOP_RIGHT(3.0),
		BOTTOM(3.0),
		BOTTOM_LEFT(3.0),
		BOTTOM_RIGHT(3.0);

		public double angle;

		CoralTarget(double angle) {
			this.angle = angle;
		}

		public double getAngle() {
			return angle;
		}
	}

	public static CoralTarget CoralTarget;

	// remember to set the CoralTarget
	public static Pose2d getCoralTargetPos(CoralTarget coralTarget) {
		Rotation2d rot = Rotation2d.fromDegrees(coralTarget.angle);
		Translation2d target = Translation2d.fromPolar(rot, kCoralDistance + kCoralDistanceOffset);
		Pose2d targetPos = Pose2d.fromTranslation(target);

		return kCoralCenter.add(targetPos);
	}

    public static Rotation2d getCoralTargetRotation(CoralTarget coralTarget) {
		return Rotation2d.fromDegrees(coralTarget.angle).flip();
    }

	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = blue_pose.mirrorAboutX(kFieldLength / 2.0);
		}

		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.mirrorAboutX(kFieldLength / 2.0);
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = blue_rotation.mirrorAboutX();
		}
		return blue_rotation;
	}

	public static double distanceFromAllianceWall(double x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength - x_coordinate;
		}
		return x_coordinate;
	}
}
