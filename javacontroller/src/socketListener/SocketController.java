package socketListener;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.Socket;

import controller.AbstractController;
import controller.ControllerManager;

/**
 * @author Rocco Gwizdziel
 *
 */
public class SocketController implements Runnable {

	protected SocketInputListener listener;

	protected String value = "";

	protected BufferedReader socketReader;

	protected BufferedWriter socketWriter;
	
	protected AbstractController abstract_controller;
	
	protected Socket server;
	
	protected boolean isClose = false;
	
	
	
	/**
	 * Konstruktor
	 * @param serv
	 * @param port
	 * @param abstract_controller
	 */
	public SocketController(String serv, int port,AbstractController abstract_controller,String name) {
		
		
		this.abstract_controller = abstract_controller;
		
		
		
		while(true){
			
		
		try {
			server = new Socket(serv, port);
            socketReader = new BufferedReader(new InputStreamReader (server.getInputStream()));
			socketWriter = new BufferedWriter(new OutputStreamWriter(server.getOutputStream()));
			
			abstract_controller.setStatus("simulation started ...");
			break;

		} catch (IOException e) {
			
			try{server.close();}catch(Exception e1){}
			abstract_controller.setStatus("... waiting for simulation");
			
		}
		try{Thread.sleep(1000);}catch(Exception e){}
		}
	}

		
	public void closeSocket(){
		try{server.close(); }catch(Exception e1){System.out.println("Socket kann nicht geschlossen werden");}
	}
	/**
	 * SocketListener anmelden
	 * @param listener
	 */
	public void addSocketInputListener(SocketInputListener listener) {
		this.listener = listener;
	}

	/**
	 * Diese Methode wird bei einer eingehenden Nachricht über den Socket aufgerufen
	 * @param event
	 */
	protected synchronized void notifyInput(SocketEvent event) {
		listener.socketGetsInput(event);

	}
	
	
	
	/**
	 * Versendet eine Nachricht über den Socket
	 * @param message
	 */
	public synchronized void sendMessage(String message) {
		
		value = message + "\0\r\n";
		try {
			socketWriter.flush();
			socketWriter.write(value);
			socketWriter.flush();
		} catch (Exception e) {
			
			System.err.println("Socket-Fehler beim Senden");
			}
			
	}

	

	/**
	 * SocketController-Thread wird gestartet
	 */
	public void startContolling() {
		new Thread(this).start();
	}
	
	
	public void run() {
		// thread liest den Input der Sockets
		new Thread() {
			public void run() {

				
				while (true) {
					
										
				   try {
						if (socketReader.ready()) {
							String s = socketReader.readLine();
							if(s.equals("X")) ControllerManager.removeAllandClose();
							notifyInput(new SocketEvent(this, s));
						}
						sleep(0,1);
					} catch (Exception e) {}

				}

			}
		}.start();

	}
}
