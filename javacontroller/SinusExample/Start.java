import controller.ControllerManager;
import examples.SinController;

public class Start {

	
	public static void main(String[] args) {
		ControllerManager.addController( new SinController("localhost",4711,4712));
		ControllerManager.addController( new SinController("localhost",4811,4812));

	}

}
