package controller;

import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.util.Iterator;
import java.util.LinkedList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

public class ControllerManager {

	static int anzahl_controller = 0;

	static LinkedList<AbstractController> ControllerList = new LinkedList<AbstractController>();

	static JFrame Window = new JFrame();

	static Container pane = new JPanel(new GridBagLayout());

	static JScrollPane jsp = new JScrollPane(pane);

	public static void addController(AbstractController controller) {
		ControllerList.add(controller);
		controller.setPosition(anzahl_controller++);

	}

	public static void startGui() {

		javax.swing.SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				GridBagConstraints c = new GridBagConstraints();
				c.fill = GridBagConstraints.BOTH;
				c.insets = new Insets(5, 5, 5, 5);
				c.anchor = GridBagConstraints.WEST;
				int reihe = 0;
				int counter = 0;
				Iterator<AbstractController> it = ControllerList.iterator();
				while (it.hasNext()) {
					AbstractController controller = it.next();
					if (counter++ % 3 == 0)
						reihe++;
					c.gridy = reihe;
					pane.add(controller, c);
				}

				// Logo*********************************
				int h = 400;
				int b = 600;
				BufferedImage img = new BufferedImage(b, h,
						BufferedImage.TYPE_INT_RGB);
				Graphics2D g = img.createGraphics();
				g.setColor(Color.WHITE);
				g.fillRect(0, 0, b - 1, h - 1);
				g.setColor(Color.black);
				g.setFont(new Font(null, Font.BOLD, 280));
				g.drawString("LPZ", 10, 300);
				Window.setIconImage(img);
				Window.setTitle("JAVA-Controller");
				// Window.setAlwaysOnTop(true);

				Window.addWindowListener(new WindowAdapter() {
					public void windowClosing(WindowEvent we) {
						ControllerManager.removeAllandClose();
					}
				});
				pane.setBackground(Color.white);

				jsp.setPreferredSize(new Dimension(1000, 500));
				Window.getContentPane().add(jsp);
				Window.pack();
				Window.setVisible(true);
			}
		});

	}

	public static synchronized void removeAllandClose() {
		Iterator<AbstractController> it = ControllerList.iterator();
		while (it.hasNext()) {
			AbstractController controller = it.next();
			System.out.println(controller.getRobotName()
					+ ": Controller-Socket geschlossen");
			try {
				controller.getSocketController().closeSocket();
			} catch (Exception e) {
			}

			System.out.println(controller.getRobotName()
					+ ": Guilogger-Socket geschlossen");
			try {
				controller.getSocketGuilogger().closeSocket();
			} catch (Exception e) {
			}

			System.out.println(controller.getRobotName() + ": wird entfernt\n");
		}

		System.out
				.println("Java-Controller beendet\nBitte noch Simulation beenden!");
		System.exit(0);
	}

}
