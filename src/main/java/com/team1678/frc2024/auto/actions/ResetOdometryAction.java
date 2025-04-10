package com.team1678.frc2024.auto.actions;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.frc2025.RobotState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetOdometryAction implements Action {

	private final String trajectory;

	private Timer autoTimer = new Timer();
	private double correctionDelay = 1.5;
	private Command pathPlannerCommand;
	private PathPlannerPath pathPlannerPath;


	public ResetOdometryAction(String trajectory) {
		this.trajectory = trajectory;

		try {
			pathPlannerPath = PathPlannerPath.fromPathFile(trajectory);
		} catch (FileVersionException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (ParseException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void start() {
		RobotState.getInstance().resetPose(pathPlannerPath.getStartingHolonomicPose().get());
	}

	@Override
	public void update() {
		System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void done() {
		
	}
}
