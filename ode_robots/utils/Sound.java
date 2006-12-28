import javax.sound.sampled.*;
import java.io.*;

public class Sound {
 public static void main(String[] args) /*throws Exception */{
  AudioFormat format=new AudioFormat(22000.0f,16,2,true,false);
  DataLine.Info sourceInfo=new DataLine.Info(SourceDataLine.class,format);
  if(!AudioSystem.isLineSupported(sourceInfo)) {System.out.println("1"); System.exit(0);}
  SourceDataLine sourceLine=null;
  try {
   sourceLine=(SourceDataLine)AudioSystem.getLine(sourceInfo);
   sourceLine.open(format);
  } catch(LineUnavailableException lue) {System.out.println("2"); System.exit(0);}
  sourceLine.start();

  byte[] data=new byte[1024];
  int numBytesRead=0;
  do {
   try {
    numBytesRead=System.in.read(data,0,1024);
   } catch(IOException ioe) {System.out.println("3"); System.exit(0);}
   sourceLine.write(data,0,1024);
  } while(numBytesRead!=-1);
  System.exit(0);
 }
}