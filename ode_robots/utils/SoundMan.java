/*
 Converts sensor values into sound.
*/

//import javax.sound.sampled.*;
import java.io.*;

public class SoundMan {
 public static void main(String[] args) {
  SoundManipulation sm=null;
  if(args.length==0) { // discrete as standard
   sm=new SoundManipulation(1,0.7f,System.in);
  } else if(args.length>0) {
   if(args[0].equals("--help") || args[0].equals("-h")) { // help
      printUsage();
      System.exit(0);
   }else if(args[0].equals("-disc")) { // discrete
    if(args.length==2) sm=new SoundManipulation(1,new Float(args[1]).floatValue(),System.in);
    else if(args.length==1) sm=new SoundManipulation(1,0.7f,System.in);
   } else if(args[0].equals("-ampl")) { // amplitude
    if(args.length==2) sm=new SoundManipulation(2,new Float(args[1]).floatValue(),System.in);
    else if(args.length==1) sm=new SoundManipulation(2,0.7f,System.in);
   } else if(args[0].equals("-freq")) { // frequency
    if(args.length==2) sm=new SoundManipulation(3,new Float(args[1]).floatValue(),System.in);
    else if(args.length==1) sm=new SoundManipulation(3,0.7f,System.in);
   }
  }
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
 }

}