import controller.ControllerManager;

public class Start {

	
	public static void main(String[] args) {
		
		MyController C = new MyController("localhost",4711,4712);
		ControllerManager.addController( C );
	}

}
