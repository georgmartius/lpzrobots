package examples;

import controller.AbstractController;



public class SinController extends AbstractController {


		
	public SinController(String serverIP, int port1, int port2) {
		super(serverIP, port1, port2);
	}


	public void init(int AnzahlSensors, int AnzahlMotors) {
		
		addConfigurableParameter("sinerate",40);
		addConfigurableParameter("phaseshift",1);
					
	}
	
	
	public void step(double[] sensors,int anzahlSensors, int anzahlMotors) {
		
		int t = getTimestep();
		double[] motors = new double[anzahlMotors];
				
				
		
		int sinerate  = getConfigurableParameterValueAsInt("sinerate");
		int phaseshift= getConfigurableParameterValueAsInt("phaseshift");
		
		for(int i=0; i < anzahlMotors; i++){
				
			motors[i] = (float)Math.sin(t/(double)sinerate + i*phaseshift*Math.PI/2.);	
				
		}
		
		sendMotorValues(motors);
	}



	

	

	

}
