package controller;

import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Enumeration;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;

import controller.parameter.ConfigurableParameter;
import controller.parameter.ConfigurableParameterList;

/**
 * @author Rocco Gwizdziel
 * Klasse erstellt ein StatusFenster zum Controller dar
 *
 */
public class ControllerWindow extends JPanel{
	
	
	
	private JProgressBar PB = new JProgressBar(0,100);
	private Timer timer = new Timer();
	private int aktuelle_rate = 100;
	private double skiprate = 0;
	
	private int buffer_size = 50;
	private int buffer_timestep[] = new int[buffer_size];
	private int timestep_alt = 0;
	
	private int counter = 0;
	
	
	private JPanel parameter_panel = new JPanel(null);
	JScrollPane JSP_parameter = new JScrollPane();
	
	private JLabel robot  = new JLabel("unknown");
	private JLabel status_text  = new JLabel("connection will be checked ...");
	private JLabel footer_panel = new JLabel("Tool zur Diplomarbeit von Rocco Gwizdziel 2007",JLabel.CENTER);
	
	private GridBagConstraints c = new GridBagConstraints();
	
	private Container pane = this;
	
	
	JTextField eingabe[];
	AbstractController abc;
	
	
	
	
	public void startWindow(final AbstractController abc){
		
		this.abc = abc;
		
		/*addWindowListener(new WindowAdapter(){
		      public void windowClosing(WindowEvent we){
		    	  ControllerManager.removeAllandClose();
		      }
		});*/
		//setTitle(" JAVA-Controller: unknown");
		//setLocation(100,100);
		setLayout(new GridBagLayout());
		//setResizable(false);
		
		setBorder(new EtchedBorder(Color.LIGHT_GRAY,Color.LIGHT_GRAY));
	
		
		
				
		//skippanel*************************************************
		c.fill = GridBagConstraints.BOTH; 
		c.anchor = GridBagConstraints.WEST;
		c.insets = new Insets(5,5,5,5);
		
		c.gridy=0;
		c.gridx=0;
		pane.add(new JLabel("Skiprate:"),c);
		c.gridx=1;
		pane.add(PB,c);
		
		c.gridy=1;
		c.gridx=0;
		pane.add(new JLabel("Robot:"),c);
		c.gridx=1;
		pane.add(robot,c);
		
		
				
		JSP_parameter= new JScrollPane(new JPanel());
		JSP_parameter.setPreferredSize(new Dimension(300,200));
		JSP_parameter.setBorder(new TitledBorder("Configurable-Parameter:"));
				
		c.weightx = 300;
		c.weighty = 200;
		c.gridwidth = 2;
		c.gridy = 2;
		c.gridx = 0;
		c.insets = new Insets(5,5,5,5);
		pane.add(JSP_parameter,c);
		c.gridwidth = 0;
		
		//status panel
		c.gridy = 3;
		c.gridx= 0;
		c.gridwidth = 2;
		pane.add(status_text,c);
		
		
		
		c.insets = new Insets(0,0,0,0);
		c.weightx = 1.0;
		c.gridwidth = 2;
		c.gridy = 4;
		c.gridx= 0;
		footer_panel.setForeground(Color.LIGHT_GRAY);
		footer_panel.setBackground(Color.white);
		footer_panel.setOpaque(true);
		footer_panel.setBorder(new EtchedBorder(Color.LIGHT_GRAY,Color.LIGHT_GRAY));
		pane.add(footer_panel,c);
		c.gridwidth = 1;
				
		//pack();
		setVisible(true);
		
	}
	
	
	public void setPosition(int pos){
		//setLocation(10+(320*pos),100);
	}
	
	
	
	public synchronized void start(String robotName, ConfigurableParameterList cpl){
         
		 parameter_panel.setBackground(Color.white);
		 //setTitle(" JAVA-Controller: "+robotName);		
		
		 eingabe = new JTextField[cpl.size()];
		 //f�r jeden configurable Parameter
		 int pos = 0;
		 Enumeration params = cpl.elements();
		 while(params.hasMoreElements()){
			final ConfigurableParameter cp = ((ConfigurableParameter)params.nextElement());
			
			JLabel l = new JLabel(cp.getKey());
			l.setBounds(5,10+pos*30,150,25);
			l.setToolTipText("Parameter: "+cp.getKey());
			parameter_panel.add(l);
			
			
		
			eingabe[pos] = new JTextField();
		    eingabe[pos].setText(cp.getValueAsString());
		    eingabe[pos].setBounds(160,10+pos*30,40,25);
		    
		   
		    parameter_panel.add(eingabe[pos]);
		
			JButton button = new JButton("change");
			final int pos_f = pos;
			button.addActionListener(new ActionListener(){
				
				
				public void actionPerformed(ActionEvent e) {
					
					try{
						double newValue = Double.parseDouble(eingabe[pos_f].getText());
						abc.setConfigurableParameter(cp.getKey(),newValue);
					}catch(Exception e1){
						eingabe[pos_f].setText(cp.getValueAsString());
						System.err.println("Eingeben Zahl hat nicht das richtige Format!");
					}
				}
				
			});
			
			button.setBounds(205,10+pos*30,60,25);
			button.setMargin(new Insets(3,0,3,0));
			parameter_panel.add(button);
			pos++;
		}
		
		parameter_panel.setPreferredSize(new Dimension(245,10+pos*30+5));
		pane.remove(JSP_parameter);
		
		JSP_parameter= new JScrollPane(parameter_panel);
		JSP_parameter.setPreferredSize(new Dimension(300,200));
		JSP_parameter.setBorder(new TitledBorder("Configurable-Parameter:"));
		
		
		c.insets = new Insets(5,5,5,5);
		c.weightx = 300;
		c.weighty = 200;
		c.gridwidth = 2;
		c.gridy = 2;
		c.gridx = 0;
		pane.add(JSP_parameter,c);
		
		timer.scheduleAtFixedRate(tt, 1, 1);
		
		robot.setText(robotName);
		robot.repaint();
		
		setVisible(true);
	}
	
	
	
	/**
	 * Setzt den Text in der Statuszeile
	 * @param s
	 */
	public void setStatus(String s){
		status_text.setText(s);
	}
	
	
	
	/**
	 * 
	 * @param timestep
	 */
	public synchronized void addTimeStep(int timestep){
		
		buffer_timestep[counter%buffer_size] = timestep-timestep_alt;
		timestep_alt = timestep+1;
		
				
		//mittlere Abstande berechnen
		int abstand = 0;
		
		//wenn buffer gef�llt
		if(counter >= buffer_size){
			
			for(int i =0; i < buffer_size; i++){
				abstand+= buffer_timestep[i];
			}	
		
		skiprate = abstand/(double)(buffer_size);
		aktuelle_rate = 100 - (int)Math.round(abstand/(double)(buffer_size));
		
		}
					
		counter++;
				
	}
	
	
	public synchronized double getSkiprate(){
		return skiprate;
		
	}
	
	private TimerTask tt = new TimerTask(){
		public void run() {
			
		if(aktuelle_rate > 90){ PB.setForeground(Color.green); setStatus("Simulation gestartet - Skiprate: sehrgut");}
		else
		if(aktuelle_rate > 70){ PB.setForeground(Color.yellow);setStatus("Simulation gestartet - Skiprate: gut"); }
		else
		if(aktuelle_rate > 50){ PB.setForeground(Color.orange);setStatus("Simulation gestartet - Skiprate: schlecht");}
		else
		if(aktuelle_rate > 20){ PB.setForeground(Color.red); setStatus("Simulation gestartet - Skiprate: schlecht");}
		
		if(aktuelle_rate < 5) aktuelle_rate=5;
		PB.setValue(aktuelle_rate);
			
		}
		
	};




	
}
