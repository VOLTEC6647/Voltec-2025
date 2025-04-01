package com.team1678.frc2024.auto.actions;

import com.team1678.lib.requests.Request;
import com.team6647.frc2025.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandAction implements Action {
	private final Command command;
	private boolean requestAdded = false;

	public CommandAction(Command command){
		this.command = command;
	}

	@Override
	public void start() {
		Superstructure s = Superstructure.getInstance();
		command.schedule();
		requestAdded = true;
	}

	@Override
	public void update() {
		System.out.println(command.isFinished());
	}

	@Override
	public boolean isFinished() {
		return command.isFinished() && requestAdded;
	}

	@Override
	public void done() {}
}
