package com.team1678.lib.requests;

import com.team1678.lib.requests.Request;
import com.team6647.frc2025.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandRequest extends Request {
	private final Command command;

	public CommandRequest(Command command) {
		this.command = command;
	}

	@Override
	public void act() {
		command.schedule();
	}

	@Override
	public boolean isFinished() {
		return command.isFinished();
	}
}
