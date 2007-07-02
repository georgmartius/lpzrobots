/*
 Converts sensor values into sound.
 Authors: Steffen Horn, Tobias Müller
*/

import java.io.*;
import sun.misc.Signal;
import sun.misc.SignalHandler;

public class SoundMan {
 public static void main(String[] args) {
  // ignore Ctrl+C
  Signal.handle(new Signal("INT"), new SignalHandler() {
    public void handle(Signal sig) {}
   }
  );

  float param=0.05f;
  int mode=1;
  SoundManipulation sm=new SoundManipulation(mode,param,System.in);
  SoundManGUI gui=new SoundManGUI(param, mode);
  gui.setVisible(true);
  sm.setGUI(gui);

  sm.start();
  try{
   sm.join();
  } catch(InterruptedException ie) {
   System.out.println(ie.getMessage());
  }
  System.exit(0);
 }
}