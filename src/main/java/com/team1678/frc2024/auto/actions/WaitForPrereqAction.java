package com.team1678.frc2024.auto.actions;

import com.team1678.lib.requests.Prerequisite;
import com.team1678.lib.requests.Request;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.util.TimeDelayedBoolean;

public class WaitForPrereqAction implements Action {
	private final Prerequisite prerequisite;
	private final double timeoutSeconds;
	private Stopwatch stopwatch;

	public WaitForPrereqAction(Prerequisite prerequisite) {
		this(prerequisite, Double.POSITIVE_INFINITY);
	}

	public WaitForPrereqAction(Prerequisite prerequisite, double timeoutSeconds) {
		this.prerequisite = prerequisite;
		this.timeoutSeconds = timeoutSeconds;
	}

	@Override
	public void start() {
		stopwatch = new Stopwatch();
		stopwatch.start();
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return prerequisite.met() || stopwatch.getTime() > timeoutSeconds;
	}

	@Override
	public void done() {}

}

