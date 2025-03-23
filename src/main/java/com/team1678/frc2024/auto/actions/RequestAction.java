package com.team1678.frc2024.auto.actions;

import com.team1678.lib.requests.Request;
import com.team6647.frc2025.subsystems.Superstructure;

public class RequestAction implements Action {
	private final Request request;
	private boolean requestAdded = false;

	public RequestAction(Request request){
		this.request = request;
	}

	@Override
	public void start() {
		Superstructure s = Superstructure.getInstance();
		s.addRequestToQueue(request);//.request();
		requestAdded = true;
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return Superstructure.getInstance().requestsCompleted() && requestAdded;
	}

	@Override
	public void done() {}
}
