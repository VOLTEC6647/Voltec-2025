package com.team6647.frc2025.auto;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team6647.frc2025.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AutoModeSelector {
	public enum DesiredMode {
		DO_NOTHING,
		TEST_PATH_AUTO,
		CENTER_6_TEST
	}

	public enum TargetNote {
		N1,
		N2,
		N3,
		N4,
		N5
	}

	public enum TargetSpike {
		LEFT,
		CENTER,
		RIGHT
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
	private TargetNote mCachedFirstNote = TargetNote.N1;
	private TargetNote mCachedSecondNote = TargetNote.N2;
	private TargetSpike mCachedSpike = TargetSpike.LEFT;

	private Optional<AutoModeBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mFirstNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mSecondNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetSpike> mSpikeChooser = new SendableChooser<>();

	public AutoModeSelector() {
		mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
		mModeChooser.addOption("Center6", DesiredMode.CENTER_6_TEST);

		SmartDashboard.putData("Auto Mode", mModeChooser);

		mFirstNoteChooser.setDefaultOption("N1", TargetNote.N1);
		mFirstNoteChooser.addOption("N2", TargetNote.N2);
		SmartDashboard.putData("First Note", mFirstNoteChooser);

		mSecondNoteChooser.addOption("N1", TargetNote.N1);
		SmartDashboard.putData("Second Note", mSecondNoteChooser);

		mSpikeChooser.setDefaultOption("Left", TargetSpike.LEFT);
		mSpikeChooser.addOption("Center", TargetSpike.CENTER);
		SmartDashboard.putData("First Spike", mSpikeChooser);
	}

	public void updateModeCreator(boolean force_regen) {
		DesiredMode desiredMode = mModeChooser.getSelected();
		TargetNote firstNote = mFirstNoteChooser.getSelected();
		TargetNote secondNote = mSecondNoteChooser.getSelected();
		TargetSpike desiredSpike = mSpikeChooser.getSelected();

		if (desiredMode == null) {
			desiredMode = DesiredMode.DO_NOTHING;
		}
		if (mCachedDesiredMode != desiredMode
				|| mCachedFirstNote != firstNote
				|| mCachedSecondNote != secondNote
				|| mCachedSpike != desiredSpike
				|| force_regen) {
			System.out.println("Auto selection changed, updating creator: desiredMode-> " + desiredMode.name() + "//"
					+ firstNote.name() + "//" + secondNote.name() + "//" + desiredMode.name() + " Spike");
			mAutoMode = getAutoModeForParams(desiredMode, firstNote, secondNote, desiredSpike);
		}
		mCachedDesiredMode = desiredMode;
		mCachedFirstNote = firstNote;
		mCachedSecondNote = secondNote;
		mCachedSpike = desiredSpike;
	}

	private Optional<AutoModeBase> getAutoModeForParams(
			DesiredMode mode, TargetNote n_0, TargetNote n_1, TargetSpike s_0) {
		switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());

			//case THREE_NOTE_MODE_34:
				//return Optional.of(new ThreeNoteMode34());

			default:
				System.out.println("ERROR: unexpected auto mode: " + mode);
				break;
		}

		System.err.println("No valid auto mode found for  " + mode);
		return Optional.empty();
	}

	public static SendableChooser<DesiredMode> getModeChooser() {
		return mModeChooser;
	}

	public DesiredMode getDesiredAutomode() {
		return mCachedDesiredMode;
	}

	public void reset() {
		mAutoMode = Optional.empty();
		mCachedDesiredMode = null;
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
	}

	public Optional<AutoModeBase> getAutoMode() {
		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}
		return mAutoMode;
	}

	public boolean isDriveByCamera() {
		return mCachedDesiredMode == DesiredMode.DO_NOTHING;
	}
}
