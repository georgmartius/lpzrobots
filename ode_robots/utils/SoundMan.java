/*
 Converts sensor values into sound.
*/

//import javax.sound.sampled.*;
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

  float param=0.0f;
  int mode=0;
  SoundManipulation sm=null;
  if(args.length==0) { // discrete as standard
   param=0.7f; mode=1; sm=new SoundManipulation(1,0.7f,System.in);
  } else if(args.length>0) {
   if(args[0].equals("-help") || args[0].equals("-h")) {
    printUsage();
    System.exit(0);
   }else if(args[0].equals("-disc")) { // discrete
    mode=1;
    if(args.length==2) {param=new Float(args[1]).floatValue(); sm=new SoundManipulation(mode,param,System.in);}
    else if(args.length==1) {param=0.7f; sm=new SoundManipulation(1,0.7f,System.in);}
   } else if(args[0].equals("-ampl")) { // amplitude
    mode=2;
    if(args.length==2) {param=new Float(args[1]).floatValue(); sm=new SoundManipulation(mode,param,System.in);}
    else if(args.length==1) {param=0.8f; sm=new SoundManipulation(2,0.8f,System.in);}
   } else if(args[0].equals("-freq")) { // frequency
    mode=3;
    if(args.length==2) {param=new Float(args[1]).floatValue(); sm=new SoundManipulation(mode,param,System.in);}
    else if(args.length==1) {param=0.8f; sm=new SoundManipulation(3,0.8f,System.in);}
   } else {
    printUsage();
    System.exit(0);
   }
  }

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

 private static void printUsage() {
  System.out.println("Usage:");
  System.out.println("\'java SoundMan -disc param\' for discrete sound output with parameter param.");
  System.out.println("\'java SoundMan -ampl param\' for amplitude modulation.");
  System.out.println("\'java SoundMan -freq param\' for frequency modulation.");
  System.out.println("Example: \'java SoundMan -disc 0.7\'");
  System.out.println("\'java SoundMan -disc\' etc. will use standard values as parameters.");
  System.out.println("\'java SoundMan\' equals \'java SoundMan -disc\'");
  System.out.println("Explanation of param:");
  System.out.println("In discrete mode a note for every (absolute) sensor value above a certain criterion is played. This criterion is defined by param.");
  System.out.println("In amplitude mode the maximum sensor value is multiplied with param to determine the volume of the output. The output consists of a single note which is altered only by it's volume.");
  System.out.println("In frequency mode the maximum sensor value is multiplied with param to alter the tone pitch.");
 }

}