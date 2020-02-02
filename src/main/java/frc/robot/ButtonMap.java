package frc.robot;

public enum ButtonMap {
	kTelescopeAxis (0),
	kManualControlPanel (1),
	kManualHood (2),
	kManualIndexer (3);

	public final int value;

	private ButtonMap(int value) {
		this.value = value;
	}
}