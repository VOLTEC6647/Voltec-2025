package com.team1678.frc2024.auto.actions;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Drive.DriveControlState;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.Superstructure;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PathplannerAlignAction implements Action {

	private Drive mDrive = null;

	private Timer autoTimer = new Timer();
	private double correctionDelay = 1.5;
	private Command pathPlannerCommand;

	public PathplannerAlignAction() {
		

		/*
		List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
				mDrive.getPose().toLegacy(),
				//new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
				endpose
				);

				 */

				/*
		PathPlannerPath path = new PathPlannerPath(
				waypoints,
				constraints,
				null, // The ideal starting state, this is only relevant for pre-planned paths, so can
						// be null for on-the-fly paths.
				new GoalEndState(0.0, endpose.getRotation()) // Goal end state. You can set a holonomic rotation
		);
		path.preventFlipping = true;
		PathPlannerTrajectory traj = path.generateTrajectory(new edu.wpi.first.math.kinematics.ChassisSpeeds(), mDrive.getPose().getRotation().toLegacy(), mDrive.config);
		*/
		
	}

	@Override
	public void start() {
		mDrive = Drive.getInstance();
		Superstructure s = Superstructure.getInstance();
		Pose2d endpose;
		if(s.level<4){
			endpose = FieldLayout.getCoralTargetPos(Superstructure.getInstance().angles[s.coralId]).realCorals[s.subCoralId].toLegacy();
		}else{
			endpose = FieldLayout.getCoralTargetPos(Superstructure.getInstance().angles[s.coralId]).corals4[s.subCoralId].toLegacy();
		}
		//endpose = endpose.rotateBy(Rotation2d.fromDegrees(180));

		if(Robot.is_red_alliance){
			endpose = endpose.rotateBy(Rotation2d.fromDegrees(180));
		}


		PathConstraints constraints = new PathConstraints(1.5, 1.5, 2 * Math.PI, 4 * Math.PI); // The constraints for

		pathPlannerCommand = AutoBuilder.pathfindToPose(endpose, constraints,0.0);
		pathPlannerCommand.schedule();
		mDrive.setControlState(DriveControlState.VELOCITY);
	}

	@Override
	public void update() {
		//System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return pathPlannerCommand.isFinished();
	}

	@Override
	public void done() {
		mDrive.setOpenLoop(new ChassisSpeeds());
	}
}
