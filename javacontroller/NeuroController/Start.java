import controller.ControllerManager;

public class Start {

	
	public static void main(String[] args) {
		
		NeuroController C = new NeuroController("localhost",4711,4712);
		ControllerManager.addController( C );
	}

}
