package controller;

import java.util.Enumeration;

import socketListener.SocketController;
import socketListener.SocketEvent;
import socketListener.SocketInputListener;
import Jama.Matrix;
import controller.parameter.ConfigurableParameter;
import controller.parameter.ConfigurableParameterList;
import controller.parameter.InternalParameter;
import controller.parameter.InternalParameterList;

/**
 * @author Rocco Gwizdziel
 *
 */
public abstract class AbstractController extends ControllerWindow implements SocketInputListener, Runnable{

	
	private int timestep = 0;

	private SocketController socket_controller,socket_InternalParams;
		
	private ConfigurableParameterList ConfigurableParameterList;
	private InternalParameterList     InternalParameterList;

	private String RobotName = "";
	
	private String serverIP;
	private int port_controller;
	private int port_internalParams;
	
	private boolean isClose = false;
	
	
	private final AbstractController abc = this;
		
	/**
	 * Konstruktor
	 * 
	 * @param serverIP IP-des 'C++ Servers' z.b. localhost
	 * @param port_controller Port-des 'C++ Controller-Servers' z.b 4444
	 * @param port_internalParams Port-des 'C++ Guilogger-Servers' z.b 4444
	 */
	public AbstractController(String serverIP, int port_controller,int port_internalParams) {
		this.serverIP = serverIP;
		this.port_controller = port_controller;
		this.port_internalParams = port_internalParams;
		
		new Thread(this).start();
	}
	
	
	
	/**
	 * Diese Methode startet den Controller-Thread
	 */
	public void run(){
		
		
		
	    startWindow(abc);
	    ControllerManager.startGui(); 
		
		
		socket_controller = new SocketController(serverIP, port_controller,this,"Controller");
		socket_controller.addSocketInputListener(this);
		socket_controller.startContolling();
		
		socket_InternalParams = new SocketController(serverIP, port_internalParams,this,"Guilogger");
		socket_InternalParams.addSocketInputListener(this);
		socket_InternalParams.startContolling();
				
		ConfigurableParameterList = new ConfigurableParameterList();
		InternalParameterList     = new InternalParameterList();
		
		
				
		
	}
	
	
	
	/**
	 * Fügt einen InternalParameter zur Überwachung mit dem Guilogger hinzu
	 * @param key angezeigter Name
	 * @param o Auszuwertendens Objekt (int, int[], int [][], double, double[], double [][], Matrix)
	 */
	public void addInternalParameter(String key, Object o){
		InternalParameterList.addParameter(new InternalParameter(key,o));
	}
	
	
	/**
	 * Fügt eine Matrix zur Überwachung mit dem Guilogger hinzu
	 * @param key angezeigter Name
	 * @param m Matrix
	 * @param type gibt an wie die Matrix ausgewertet werden soll
	 * InternalParameter.Diagonal_and_4x4
	 * InternalParameter.MatrixNorm_MaxColSum
	 * InternalParameter.MatrixNorm_MaxRowSum
	 * InternalParameter.MatrixNorm_Frobenius
	 * InternalParameter.MatrixNorm_MaxSingularValue
	 */
	public void addInternalParameter(String key, Matrix m, int type){
		InternalParameterList.addParameter(new InternalParameter(key,m,type));
	}
	
	
	/**
	 * Aktualisiert einen Wert in der Liste der überwachten Parameter, falls vorhanden<br>
	 * der Aufruf dieser Methode ist nicht zwingend erforderlich
	 * bei primitiven Typen (int, double) muss sie aufgerufen werden
	 * @param key name des überwachten Parameter
	 * @param o   Objekt des überwachten Parameters
	 */
	public void setInternalParameter(String key, Object o){
		InternalParameterList.setParameter(key, o);
	}
	
	
	
	/**
	 * Methode fügt einen Parameter zur ConfigurableParameterList hinzu
	 * @param key Name des Parameters
	 * @param value Startwert des Parameters
	 */
	public void addConfigurableParameter(String key, double value){
		ConfigurableParameter cp = new ConfigurableParameter(key,value);
		ConfigurableParameterList.addParameter(cp);
		
	}
	
	
	/**
	 * Methode ändert den Wert eines Parameters, falls vorhanden
	 * @param key Name des Parameter
	 * @param value neuer Wert des Parameters
	 */
	public void setConfigurableParameter(String key, double value){
		System.out.println("Parameter "+key+" set on "+value);
		ConfigurableParameterList.setParameter(key,value);
	}
	
	
	
	/**
	 * Methode gibt den aktuellen Wert eines ConfigurableParameter als double zurück
	 * @param key Parametername
	 * @return aktuller Parameterwert
	 */
	public double getConfigurableParameterValueAsDouble(String key){
		ConfigurableParameter p = ConfigurableParameterList.getParameter(key);
		//bei fehler
		if(p==null){
			System.err.println("Parameter "+key+" ist nicht in ConfigurableParameterList");
			System.exit(1);
		}
		return p.getValueAsDouble();
	}
	
	
	/**
	 * Methode gibt den aktuellen Wert eines ConfigurableParameter als int zurück
	 * @param key Parametername
	 * @return aktuller Parameterwert
	 */
	public int getConfigurableParameterValueAsInt(String key){
		ConfigurableParameter p = ConfigurableParameterList.getParameter(key);
		//bei fehler
		if(p==null){
			System.err.println("Parameter "+key+" ist nicht in ConfigurableParameterList");
			System.exit(1);
		}
		return p.getValueAsInt();
	}
	
	
	/**
	 * Methode versendet alle gesetzten Parameter<br>
	 * Der Java-Controller auf der 'c++ Seite' wartet auf diesen aufruf
	 */
	private void sendInitialisation(){
		
		String send = "I#";
		
		//ConfigurableParameterList
		//anzahl#p1_Key#p2_Value ... #pn_Key#pn_Value
		int size = ConfigurableParameterList.size();
		send += size;
		
		if(size > 0){
			Enumeration params = ConfigurableParameterList.elements();
			while(params.hasMoreElements()){
				ConfigurableParameter p = (ConfigurableParameter)params.nextElement();
				send += "#"+p.getKey()+"#"+p.getValueAsDouble();
			}
		}
	    //System.out.println(send);
		socket_controller.sendMessage(send);
		
		
		//hier werden die InternalParameters abgefragt
		String internalParameter = "GI";
		internalParameter+= InternalParameterList.getAllNames();
		socket_InternalParams.sendMessage(internalParameter);
		//System.out.println(internalParameter);
		
		
				
		
		 javax.swing.SwingUtilities.invokeLater(new Runnable() {
	            public void run() {
	            	abc.start(RobotName+" (Port:"+port_controller+")",ConfigurableParameterList);
	            	
	            }
	        });
		
		
		
	}
	
	/**
	 * Methode versendet MotorValues und die Werte für den Guilogger
	 * @param motors aktuelle Motorvalues
	 */
	public void sendMotorValues(double[] motors) {
		String send = "N#";
		
		//motorwerte
		for (int i = 0; i < motors.length; i++) {
			send += motors[i] + "#";
		}
		
		socket_controller.sendMessage(send);
		sendGuiloggerValues();
	}
	
	
	
	/**
	 * Methode versendet MotorValues und die Werte für den Guilogger
	 * @param motors aktuelle Motorvalues als nx1(1xn) Matrix
	 */
	public void sendMotorValues(Matrix motors) {
		double motorsArray[] = motors.getColumnPackedCopy();
		String send = "N#";
		
		//motorwerte
		for (int i = 0; i < motorsArray.length; i++) {
			send += motorsArray[i] + "#";
		}
		
		socket_controller.sendMessage(send);
		sendGuiloggerValues();
	}
	
	
	/**
	 * Versendet die aktuellen Werte der überwachten Parameter an den Guilogger
	 *
	 */
	private void sendGuiloggerValues(){
		String internalParameter = "GD";
		internalParameter+= InternalParameterList.getAllValues();
		socket_InternalParams.sendMessage(internalParameter);
		
	}

	/**
	 * Methode wird aufgerufen wenn der Socket einen Input hat
	 * @param SocketEvent event
	 */
	public void socketGetsInput(SocketEvent event) {

		//wenn initialisierungs event empfangen wird (aufruf init() )
		if (event.getEventMode()== SocketEvent.Initialisation){
		    	System.out.println("Java-Controller wurde initialisiert\n");
				
		    	init(event.getAnzahlSensors(), event.getAnzahlMotors());
				
				RobotName = event.getRobotName();
				
				sendInitialisation();
				
		}else 
		
		//wenn normaler sensor-motor-steuerungs-event (aufruf StepNoLearning() )
		if(event.getEventMode()== SocketEvent.StepNoLearning){
			timestep = event.getTimestep();
			addTimeStep(timestep);
		   	step(event.getSensors(),event.getAnzahlSensors(), event.getAnzahlMotors());
			
		}else
			
		//wenn ConfigurableParameterSettings event (aufruf setParam(const paramkey& key, paramval val) )
		if(event.getEventMode()== SocketEvent.ConfigurableParameterSettings){
			//setzten bzw. ändern des Parameters
			setConfigurableParameter(event.getParamKey(),Double.parseDouble(event.getParamValue()));
		}
		
		
	}
	
		
	public SocketController getSocketController(){
		return socket_controller;
	}
	
	public SocketController getSocketGuilogger(){
		return socket_InternalParams;
	}
	
	public String getRobotName(){
		return RobotName;
	}
	
	
	/**
	 * @return Aktueller Zeitschritt der Simulation
	 */
	public int getTimestep() {
		return timestep;
	}
	
	/**
	 * @return ob der AbstractController geschlossen wurde
	 */
	public boolean isClose(){
		return isClose;
	}
	
	/**
	 * Abstrakte Methode, wird beim Starten der Simulation aufgerufen
	 * @param anzahlSensors
	 * @param anzahlMotors
	 */
	public abstract void init(int anzahlSensors,int anzahlMotors);
	/**
	 * Abstrakte Methode, wird in jedem Zeitschritt der Simulation aufgerufen
	 * @param sensors
	 * @param anzahlSensors
	 * @param anzahlMotors
	 */
	public abstract void step(double[] sensors,int anzahlSensors,int anzahlMotors);
}
