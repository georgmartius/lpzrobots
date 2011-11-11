/*
 Converts sensor values into sound.
 Authors: Steffen Horn, Tobias Mueller
*/

public class SoundMan {
 public static void main(String[] args) {
  // ignoring Ctrl+C is now done with the shell script

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