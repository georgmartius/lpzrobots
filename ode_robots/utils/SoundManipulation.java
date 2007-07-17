/*
 Converts sensor values into sound.
*/

import javax.sound.midi.*;
import java.io.*;
import java.util.regex.*;

public class SoundManipulation extends Thread {
 private int mode;
 private float param;
 private int numSensors;
 private InputStream is;
 private Receiver synthRcvr;
 private Synthesizer synth;
 private SoundManGUI gui;
 private String[] instrNames;
 private float[][] oldSensorValues;
 private int oldNote[];
 private int oldVolume[];

 public SoundManipulation(int mode, float param, InputStream is) {
  this.mode=mode;
  this.param=param;
  this.is=is;

  // initialize sound output
  Sequencer seq;
  Transmitter seqTrans;
  try {
   seq=MidiSystem.getSequencer();
   seqTrans=seq.getTransmitter();
   synth=MidiSystem.getSynthesizer();
   synth.open();
   Soundbank sb = synth.getDefaultSoundbank();
   Instrument[] instruments=sb.getInstruments();
   instrNames=new String[instruments.length];
   for(int i=0; i<instrNames.length; i++) instrNames[i]=instruments[i].getName();
   synthRcvr=synth.getReceiver();
   seqTrans.setReceiver(synthRcvr);
  } catch(MidiUnavailableException mue) {
   System.out.println(mue.getMessage());
   System.exit(0);
  }
 }

 public void setGUI(SoundManGUI gui) {
  this.gui=gui;
  this.gui.setInstruments(instrNames);
 }

 public void run() {
  StringBuilder input=new StringBuilder();
  float sensorDiff=0.0f;
  long time=0;
  float[] sensorDiffs=null;
  int next=0;
  int index=-1;
  boolean initIsOver=false;
  String[] values=null;
  while(true) {
    try {
     next=is.read(); // read next input as byte
    } catch(IOException ioe) {
     System.out.println(ioe.getMessage());
     System.exit(0);
    }
    if(next==-1) break; // end of input stream reached
    // parse input
    if(index<2*numSensors) input.append((char)next);
    if(initIsOver && next==' ' && index<2*numSensors) {
     if(index>=0) {
      if(index>=numSensors) values[index-numSensors]=input.toString();
      input.delete(0,input.length());
     }
     index++;
    }
    if(next=='\n') {
     index=-1;
     if(input.indexOf("#C")==0) {
      // find the number of sensors of the robot
      Pattern p = Pattern.compile("x\\[\\d++\\]");
      Matcher m = p.matcher(input);
      m.find();
      do {
       numSensors=Integer.parseInt(m.group().substring(2,m.group().length()-1))+1;
      } while(m.find());
      oldSensorValues=new float[numSensors][2];
      sensorDiffs=new float[numSensors];
      oldNote=new int[numSensors];
      oldVolume=new int[numSensors];
      values=new String[numSensors];
      for(int i=0; i<numSensors; i++) values[i]="0.0";
     } else if(Math.abs(System.currentTimeMillis()-time)>gui.getToneLength()) {
      // new data received and timestep is over
      time=System.currentTimeMillis();
      if(input.indexOf("#")==-1) {
       initIsOver=true;
       // set the number of sensors of the robot and handle sensor values
       gui.setNumSensors(numSensors);
       float sensorMax=0.0f;
       param=gui.getParam();
       if(mode!=gui.getMode()) {
        mode=gui.getMode();
        synth.getChannels()[0].allNotesOff();
       }
       if(gui.instrumentChanged()) {
        // change instrument for playback
        clearOldSensorValues();
        synth.getChannels()[0].allNotesOff();
        synth.getChannels()[0].programChange(gui.getInstrument());
       }
       updateOldSensorValues();
       if(gui.isPlaybackActive()) {
        for(int i=0; i<numSensors; i++) {
         // smoothing the changes of sensor values
         oldSensorValues[i][1]=new Float(values[i]).floatValue();
         sensorDiff=0.5f*(oldSensorValues[i][1]-oldSensorValues[i][0]);
         if(gui.isSmoothingEnabled()) {
          // smoothing enabled
          sensorDiffs[i]+=0.1f*(sensorDiff-sensorDiffs[i]);
          sensorDiff=Math.abs(sensorDiffs[i]);
         } else {
          // smoothing disabled
          sensorDiff=Math.abs(sensorDiff);
         }
         gui.setSensorValue(i,sensorDiff);
         if(gui.isSensorChecked(i) || i==numSensors-1) {
          switch(mode) {
           case 1: // discrete
            if(sensorDiff>param && gui.isSensorChecked(i)) {
             int note=gui.getNote()+27*i/numSensors;
             play(i,note,90);
            }
            break;
           case 2: // amplitude
            if(gui.isSensorChecked(i)) sensorMax=Math.max(sensorMax,sensorDiff);
            if(i==numSensors-1) {
             int note=gui.getNote()+27;
             int volume=(int)Math.min(127,param*sensorMax*12700.0f);
             play(i,note,volume);
             sensorMax=0.0f;
            }
            break;
           case 3: // frequency
            if(gui.isSensorChecked(i)) sensorMax=Math.max(sensorMax,sensorDiff);
            if(i==numSensors-1) {
             int note=(int)Math.min(127,param*sensorMax*1270.0f);
             play(i,note,90);
             gui.setNote(note);
             sensorMax=0.0f;
            }
            break;
           case 4: // master mode
            if(sensorDiff>0.0f && gui.isSensorChecked(i)) {
             int volume=(int)Math.min(100,sensorDiff*250.0f);
             int note=volume+27*i/(numSensors-1);
             play(i,note,volume);
            }
           break;
          }
         }
        }
       } else {
        // mute playback
        synth.getChannels()[0].allNotesOff();
       }
      }
     }
     input.delete(0,input.length());
   }
  }
 }

 private void play(int i, int note, int volume) {
  try {
   ShortMessage sm=new ShortMessage();
   sm.setMessage(ShortMessage.NOTE_OFF, 0, oldNote[i], oldVolume[i]);
   synthRcvr.send(sm, -1);
   sm.setMessage(ShortMessage.NOTE_ON, 0, note, volume);
   synthRcvr.send(sm, -1);
   oldNote[i]=note;
   oldVolume[i]=volume;
  } catch(InvalidMidiDataException imde) {
   System.out.println(imde.getMessage());
   System.exit(0);
  }
 }

 private void clearOldSensorValues() {
  for(int i=0; i<oldSensorValues.length; i++) {
   oldSensorValues[i][0]=0.0f;
   oldSensorValues[i][1]=0.0f;
  }
 }

 private void updateOldSensorValues() {
  for(int i=0; i<oldSensorValues.length; i++) oldSensorValues[i][0]=oldSensorValues[i][1];
 }

}