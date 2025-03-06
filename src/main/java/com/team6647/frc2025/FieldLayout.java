package com.team6647.frc2025;

import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team6647.frc2025.auto.modes.configuredQuals.putCoral;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;
import java.util.List;
import java.util.Vector;

import org.littletonrobotics.junction.Logger;

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
	public static double kFieldLength = Units.inchesToMeters(690.876);
	public static double kFieldWidth = Units.inchesToMeters(317);
	public static double kStartingLineX = Units.inchesToMeters(299.438);

	public static final double kApriltagWidth = Units.inchesToMeters(6.50);
	public static final AprilTagFieldLayout kTagMap;

	static {
		//try {
			kTagMap = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
			//kTagMap = new AprilTagFieldLayout("/edu/wpi/first/apriltag/2024-crescendo.json");
			//kTagMap = new AprilTagFieldLayout("/edu/wpi/first/apriltag/2025-reefscape.json");
		//} catch (IOException e) {
		//	throw new RuntimeException(e);
		//}
	}
	
	public static Pose2d kCoralCenter;
	static{
		if(Robot.isReal()&&DriverStation.getAlliance().get() == Alliance.Red){
			kCoralCenter = new Pose2d((kTagMap.getTagPose(19).get().getX()+kTagMap.getTagPose(20).get().getX())/2,kTagMap.getTagPose(18).get().getY(),new Rotation2d());
		}else{
			kCoralCenter = new Pose2d((kTagMap.getTagPose(19).get().getX()+kTagMap.getTagPose(20).get().getX())/2,kTagMap.getTagPose(18).get().getY(),new Rotation2d());
		}
	}

	public static double kCoralDistance = 1.6; //Please Fix
	public static double kCoralDistanceOffset = 0.3f; // temproral
	public static double kPreDistance = 1.0f;

	public enum CoralTarget {
		RIGHT(0.0),
		BOTTOM_RIGHT(60.0),
		BOTTOM_LEFT(120.0),
		LEFT(180.0),
		TOP_LEFT(240.0),
		TOP_RIGHT(300.0);
			
		public Angle angleA;

		public double angle;

		CoralTarget(double angle) {
			this.angle = angle;
		}
	}

	public static CoralTarget CoralTarget;

	public static class CoralSet{
		public Pose2d algae;
		public Pose2d coral1;
		public Pose2d coral2;
		public Pose2d[] corals;

		public CoralSet(Pose2d algae, Pose2d coral1, Pose2d coral2, Pose2d algaePre, Pose2d coral1Pre, Pose2d coral2Pre, Pose2d[] corals) {
			this.algae = algae;
			this.coral1 = coral1;
			this.coral2 = coral2;
			//this.algaePre = algaePre;
			//this.coral1Pre = coral1Pre;
			//this.coral2Pre = coral2Pre;
			this.corals = corals;
		}
	}

	static{
		//Logger.recordOutput("/Coral/Center", kCoralCenter.toLegacy()); LEGACY
	}

	public static CoralSet getCoralTargetPos(CoralTarget coralTarget) {
		Rotation2d rot = Rotation2d.fromDegrees(coralTarget.angle).inverse();
		
		//Pose2d center = new Pose2d(kCoralCenter.getTranslation().x()+kCoralDistance, kCoralCenter.getTranslation().y(), new Rotation2d(180,true));
		Pose2d center = new Pose2d(kCoralCenter.getTranslation().x()+kCoralDistance, kCoralCenter.getTranslation().y(),new Rotation2d());


		Pose2d algae = center.transformBy(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
		Pose2d coral1 = center.transformBy(new Pose2d(new Translation2d(0, kCoralDistanceOffset), new Rotation2d()));
		Pose2d coral2 = center.transformBy(new Pose2d(new Translation2d(0, -kCoralDistanceOffset), new Rotation2d()));
		
		Pose2d algaePre = algae.transformBy(new Pose2d(new Translation2d(kPreDistance, 0), new Rotation2d()));
		Pose2d coral1Pre = coral1.transformBy(new Pose2d(new Translation2d(kPreDistance, 0), new Rotation2d()));
		Pose2d coral2Pre = coral2.transformBy(new Pose2d(new Translation2d(kPreDistance, 0), new Rotation2d()));
		
		algae = rotatePoseFromPivot(algae, rot);
		coral1 = rotatePoseFromPivot(coral1, rot);
		coral2 = rotatePoseFromPivot(coral2, rot);

		algae = handleCoralFlip(algae.rotateBy(Rotation2d.fromDegrees(180)),Robot.is_red_alliance);
		coral1 = handleCoralFlip(coral1.rotateBy(Rotation2d.fromDegrees(180)),Robot.is_red_alliance);
		coral2 = handleCoralFlip(coral2.rotateBy(Rotation2d.fromDegrees(180)),Robot.is_red_alliance);

		algaePre = new Pose2d();//rotatePoseFromPivot(algaePre, rot);
		coral1Pre = new Pose2d();//rotatePoseFromPivot(coral1Pre, rot);
		coral2Pre = new Pose2d();//rotatePoseFromPivot(coral2Pre, rot);

		Pose2d[] corals = {coral1, coral2};

		return (new CoralSet(algae, coral1, coral2, algaePre, coral1Pre, coral2Pre, corals));
	}

	public static Pose2d rotatePoseFromPivot(Pose2d coral, Rotation2d rot) {
		return rotatePose2dFromPivot(coral, kCoralCenter.getTranslation(), rot);
	}
	
	public static Pose2d rotatePose2dFromPivot(Pose2d pose, Translation2d pivot, Rotation2d angle) {
        Translation2d translated = pose.getTranslation().minus(pivot);
        Translation2d rotated = translated.rotateBy(angle);
        Translation2d finalTranslation = rotated.plus(pivot);
        Rotation2d finalRotation = pose.getRotation().add(angle);
        return new Pose2d(finalTranslation, finalRotation);
    }

    public static Rotation2d getCoralTargetRotation(CoralTarget coralTarget) {
		return Rotation2d.fromDegrees(coralTarget.angle).flip();
    }

	public static Pose2d handleCoralFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = blue_pose.mirrorAboutX(kFieldLength / 2.0).mirrorAboutY(kFieldWidth / 2.0);
		}

		return blue_pose;
	}

	public static Pose2d[] coralStations = {
		//Left Center
		new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            //Rotation2d.fromDegrees(90 - 144.011)),
            Rotation2d.fromDegrees(90)),


		//Right Center
		new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            //Rotation2d.fromDegrees(90 - 144.011)),
            Rotation2d.fromDegrees(90)),
	};

	public static Pose2d getCoralStation(boolean bottomStation,int positionId){
		Pose2d coralStation = coralStations[positionId];
		if(bottomStation){
			coralStation = coralStation.mirrorAboutY(kFieldWidth / 2.0);
		}
		coralStation = handleAllianceFlip(coralStation, Robot.is_red_alliance);
		return coralStation;
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
