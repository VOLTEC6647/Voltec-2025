package com.team6647.frc2025.auto.paths;

import java.util.ArrayList;
import java.util.List;

import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.swerve.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.subsystems.Superstructure;

import edu.wpi.first.math.util.Units;

public class TrajectoryGenerator {

	private static TrajectoryGenerator mInstance;

	public static TrajectoryGenerator getInstance() {
		if (mInstance == null) {
			mInstance = new TrajectoryGenerator(Drive.getInstance().getMotionPlanner());
		}
		return mInstance;
	}

	private static final double kMaxAccel = 6.0;
	private static final double kMaxVoltage = 9.0;

	private final DriveMotionPlanner mMotionPlanner;
	private TrajectorySet mTrajectorySet = null;

	public TrajectoryGenerator(DriveMotionPlanner motion_planner) {
		mMotionPlanner = motion_planner;
	}

	public void generateTrajectories() {
		if (mTrajectorySet == null) {
			System.out.println("Generating trajectories...");
			mTrajectorySet = new TrajectorySet();
			System.out.println("Finished trajectory generation");
		}
	}

	public void forceRegenerateTrajectories(boolean mirror) {
		System.out.println("Generating trajectories..." + (mirror ? " MIRRORED VERSION" : ""));
		mTrajectorySet = new TrajectorySet(mirror);
		System.out.println("Finished trajectory generation");
	}

	public TrajectorySet getTrajectorySet() {
		return mTrajectorySet;
	}

	public Trajectory254<TimedState<Pose2dWithMotion>> generateTrajectory(
			boolean reversed,
			final List<Pose2d> waypoints,
			final List<Rotation2d> headings,
			final List<TimingConstraint<Pose2dWithMotion>> constraints,
			double max_vel, // m/s
			double max_accel, // m/s^2
			double max_voltage) {
		return mMotionPlanner.generateTrajectory(
				reversed, waypoints, headings, constraints, max_vel, max_accel, max_voltage);
	}

	public Trajectory254<TimedState<Pose2dWithMotion>> generateTrajectory(
			boolean reversed,
			final List<Pose2d> waypoints,
			final List<Rotation2d> headings,
			final List<TimingConstraint<Pose2dWithMotion>> constraints,
			double start_vel, // m/s
			double end_vel, // m/s
			double max_vel, // m/s
			double max_accel, // m/s^2
			double max_voltage) {
		return mMotionPlanner.generateTrajectory(
				reversed, waypoints, headings, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
	}

	public class TrajectorySet {

		public final Trajectory254<TimedState<Pose2dWithMotion>> testT;
		public Trajectory254<TimedState<Pose2dWithMotion>> putCoral;
		public Trajectory254<TimedState<Pose2dWithMotion>> forward;



		private boolean wants_mirrored;

		private TrajectorySet(boolean mirror) {
			wants_mirrored = mirror;
			// spotless:off
			testT = getTestTrajectory();
			// spotless:on
		}

		private TrajectorySet() {
			this(false);
		}

		private void convertToM(List<Pose2d> waypoints, List<Rotation2d> headings) {
			for (int i = 0; i < waypoints.size(); ++i) {
				System.out.println("waypoints.add(new Pose2d("
						+ Units.inchesToMeters(waypoints.get(i).getTranslation().x())
						+ ", "
						+ Units.inchesToMeters(waypoints.get(i).getTranslation().y())
						+ ", Rotation2d.fromDegrees("
						+ waypoints.get(i).getRotation().getDegrees()
						+ ")));");
				System.out.println(
						"headings.add(Rotation2d.fromDegrees(" + headings.get(i).getDegrees() + "));");
			}
			System.out.println("\n\n");
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> generate(
				List<Pose2d> waypoints,
				List<Rotation2d> headings,
				List<TimingConstraint<Pose2dWithMotion>> constraints,
				boolean reversed,
				double percentSpeed,
				double percentAccel) {

			handleAllianceFlip(waypoints, headings);
			return generateTrajectory(
					reversed,
					waypoints,
					headings,
					constraints,
					percentSpeed * Constants1678.SwerveConstants.maxAutoSpeed,
					percentAccel * kMaxAccel,
					kMaxVoltage);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> generateD(
				List<Pose2d> waypoints,
				List<Rotation2d> headings,
				List<TimingConstraint<Pose2dWithMotion>> constraints,
				boolean reversed,
				double percentSpeed,
				double percentAccel) {

			//handleAllianceFlip(waypoints, headings);
			return generateTrajectory(
					reversed,
					waypoints,
					headings,
					constraints,
					percentSpeed * Constants1678.SwerveConstants.maxAutoSpeed,
					percentAccel * kMaxAccel,
					kMaxVoltage);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> generate(
				List<Pose2d> waypoints,
				List<Rotation2d> headings,
				List<TimingConstraint<Pose2dWithMotion>> constraints,
				boolean reversed,
				double percentSpeed,
				double percentAccel,
				double start_vel,
				double end_vel) {
			handleAllianceFlip(waypoints, headings);
			Trajectory254<TimedState<Pose2dWithMotion>> trajectory = generateTrajectory(
					reversed,
					waypoints,
					headings,
					constraints,
					start_vel * Constants1678.SwerveConstants.maxAutoSpeed,
					end_vel * Constants1678.SwerveConstants.maxAutoSpeed,
					percentSpeed * Constants1678.SwerveConstants.maxAutoSpeed,
					percentAccel * kMaxAccel,
					kMaxVoltage);
			return trajectory;
		}

		private void handleAllianceFlip(List<Pose2d> waypoints, List<Rotation2d> headings) {
			if (wants_mirrored) {
				for (int i = 0; i < waypoints.size(); i++) {
					waypoints.set(
							i,
							new Pose2d(
									waypoints.get(i).getTranslation().mirrorAboutX(FieldLayout.kFieldLength / 2.0),
									waypoints.get(i).getRotation().mirrorAboutX()));
				}
				for (int i = 0; i < headings.size(); i++) {
					headings.set(i, headings.get(i).mirrorAboutX());
				}
			}
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getTestTrajectory() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.5, 1.5, Rotation2d.fromDegrees(0)));
			headings.add(Rotation2d.fromDegrees(180.0));
			waypoints.add(new Pose2d(7.0, 1.5, Rotation2d.fromDegrees(0)));
			headings.add(Rotation2d.fromDegrees(180.0));
			return generate(waypoints, headings, List.of(), false, 1, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5S1PickupToTeleStart() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.6, 6.8, Rotation2d.fromDegrees(-110.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.6, 0.7, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.6, 0.7, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}


		public Trajectory254<TimedState<Pose2dWithMotion>> getPutCoral() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(Drive.getInstance().getPose());
			headings.add(Rotation2d.fromDegrees(Drive.getInstance().getPose().getRotation().getDegrees()));

			waypoints.add(FieldLayout.getCoralTargetPos(Superstructure.getInstance().angles[Superstructure.getInstance().coralId]).realCorals[Superstructure.getInstance().subCoralId]);
			headings.add(Rotation2d.fromDegrees(FieldLayout.getCoralTargetPos(Superstructure.getInstance().angles[Superstructure.getInstance().coralId]).realAlgae.getRotation().getDegrees()));
			return generateD(waypoints, headings, List.of(), false, 0.4, 1.0);
		}

		public Trajectory254<TimedState<Pose2dWithMotion>> getForward() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();

			waypoints.add(new Pose2d(7.85860538482666,3.842921018600464,Rotation2d.fromDegrees(0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(5.757699012756348,3.842921018600464,Rotation2d.fromDegrees(0)));
			
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}
	

		public Trajectory254<TimedState<Pose2dWithMotion>> goToSource1() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();

			Pose2d bottomSource = new Pose2d(0.754292, 6.741, new Rotation2d(0.0, true));
			Pose2d topSource = new Pose2d(1.717, 7.409, new Rotation2d(0.0, true));

			if (bottomSource.distance(Drive.getInstance().getPose()) < topSource.distance(Drive.getInstance().getPose())) {
                waypoints.add(new Pose2d(0.754292, 6.741, new Rotation2d(0.0, true)));
			} else {
				waypoints.add(new Pose2d(1.717, 7.409, new Rotation2d(0.0, true)));
			}
			headings.add(new Rotation2d(2.24, true));

			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}

		public Trajectory254<TimedState<Pose2dWithMotion>> goToSource2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();

			Pose2d bottomSource = new Pose2d( 0.754292, 0.6854, new Rotation2d(0.0, true));
			Pose2d topSource = new Pose2d(0.7355, 1.33475, new Rotation2d(0.0, true));

			if (bottomSource.distance(Drive.getInstance().getPose()) < topSource.distance(Drive.getInstance().getPose())) {
                waypoints.add(bottomSource);
			} else {
				waypoints.add(topSource);
			}
			headings.add(new Rotation2d(-2.2348, true));

			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}
    }
}
