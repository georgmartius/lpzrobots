import controller.AbstractController;



public class MyController extends AbstractController {


		
	public MyController(String serverIP, int port_controller, int port_internalParams) {
		super(serverIP, port_controller, port_internalParams);
	}


	public void init(int AnzahlSensors, int AnzahlMotors) {
		
		//hier kann initialisiert werden
		//z.B. addConfigurableParameter("CP",0.5)
			
	}
	
	
	public void step(double[] sensors,int anzahlSensors, int anzahlMotors) {
		
		int t = getTimestep();
		double[] motors = new double[anzahlMotors];
		
		// !!! hier beginnt der wichtigste Teil des Controllers !!!
		//berechnen der neuen Motorwerte anhand
		//der Sensorwerte
		
		
		//Uebergabe der neuen Motorwerte
		// an den Use-JAVA-Controller (die Simulation)
		sendMotorValues(motors);
	}

}

