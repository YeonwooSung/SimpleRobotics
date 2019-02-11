import easyGui.EasyGui;

public class Robot {
	private final EasyGui gui;

	/**
	 * This constructor sets up the basic settings of the EasyGui instance.
	 */
	public Robot() {
		gui = new EasyGui(100,100);
		
		gui.addLabel(0, 0, "Select an algorithm: ");

		gui.addButton(1, 0, "RRT", this, "rrt");
		
		gui.addButton(1, 1, "Potential Fields", this, "potentialFields");
	}

	/**
	 * This method displays the GUI to run the project.
	 */
	public void runProject() {
		gui.show();
	}

	/**
	 * Runs the robot with RRT type instance.
	 */
	public void rrt() {
		gui.hide();
		RRT rt = new RRT();
		rt.runRobot();
	}

	/**
	 * Runs the robot with PotentialFields type instance.
	 */
	public void potentialFields() {
		gui.hide();
		PotentialFields fields = new PotentialFields();
		fields.runRobot();
	}

	// MAIN
	public static void main(String[] args) {
		Robot robot = new Robot();
		robot.runProject();
	}
}
