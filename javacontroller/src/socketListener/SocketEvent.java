package socketListener;

import java.util.EventObject;


/**
 * @author Rocco Gwizdziel
 *
 */
public class SocketEvent extends EventObject{

	public final static int StepNoLearning				    = 0;
	public final static int ConfigurableParameterSettings 	= 1;
	public final static int Initialisation				 	= 3;
	
	  
	String data;
	
	int eventMode = 0;
	
	String parsedData[];
	
	public SocketEvent(Object arg0, String data) {
		super(arg0);
		
		//I#timestep#anz_sensor#anz_motor#sens1&sens2&..sensN#moto1&moto2&...motoN
		this.data = data;
		parsedData = data.split("#");
		
		if(data.startsWith("C")) eventMode= ConfigurableParameterSettings;
		if(data.startsWith("N")) eventMode= StepNoLearning;
		if(data.startsWith("I")) eventMode= Initialisation;
		
	}

	/**
	 * @return Returns the data.
	 */
	public String getData() {
		return data;
	}
	
	public double[] getMotors(){
		String s[] = parsedData[5].split("&"); 
		double  f[] = new double[getAnzahlMotors()];
		for(int i=0; i < getAnzahlMotors();i++){
			f[i] = Double.parseDouble(s[i]);
		}
		
		return f;
	}
	
	public double[] getSensors(){
		String s[] = parsedData[4].split("&"); 
		double  f[] = new double[getAnzahlSensors()];
		
		for(int i=0; i < getAnzahlSensors();i++){
			f[i] = Double.parseDouble(s[i]);
		}
		
		return f;
	}
	
	
	
	public String getParamKey(){
		return parsedData[1];
	}
	
	public String getParamValue(){
		return parsedData[2];
	}
	
	
	
	public int getTimestep(){
		return Integer.parseInt(parsedData[1]);
	}
	
	public int getAnzahlMotors(){
		return Integer.parseInt(parsedData[3]);
	}
	
	public int getAnzahlSensors(){
		return Integer.parseInt(parsedData[2]);
	}
	
	public String getRobotName(){
		return parsedData[4];
	}

	
	
	
	public int getEventMode() {
		return eventMode;
	}
	
	
	
	

}
