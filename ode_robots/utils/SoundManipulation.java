/*
 Converts sensor values into sound.
*/

import javax.sound.midi.*;
import java.io.*;

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

 public SoundManipulation(int mode, float param, InputStream is) {
  this.mode=mode;
  this.param=param;
  this.is=is;

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
  String input="";
  float sensorDiff=0.0f;
  while(true) {
   int next=0;
   try {
    next=is.read();
   } catch(IOException ioe) {
    System.out.println(ioe.getMessage());
    System.exit(0);
   }
   if(next==-1) break;
   input+=(char)next;
   if((char)next=='\n') {
    if(input.startsWith("#N neuron x[")) {
     numSensors=input.charAt(12)-47;
     oldSensorValues=new float[numSensors][2];
    } else if(!input.startsWith("#")) {
     gui.setNumSensors(numSensors);
     String[] values=input.trim().substring(0,numSensors*10).split(" ");
     float sensorMax=0.0f;
     param=gui.getParam();
     mode=gui.getMode();
     if(gui.instrumentChanged()) {
      synth.getChannels()[0].allNotesOff();
      synth.getChannels()[0].programChange(gui.getInstrument());
     }
     updateOldSensorValues();
     if(gui.isPlaybackActive()) {
      for(int i=0; i<numSensors; i++) {
       oldSensorValues[i][1]=Math.abs(new Float(values[i]).floatValue());
       sensorDiff=Math.abs(oldSensorValues[i][1]-oldSensorValues[i][0]);
       gui.setSensorValue(i,sensorDiff);
       switch(mode) {
        case 1: // discrete
         if(sensorDiff>param) {
          try {
           ShortMessage sm=new ShortMessage();
           sm.setMessage(ShortMessage.NOTE_ON, 0, gui.getNote()+27*i/numSensors, 90);
           synthRcvr.send(sm, -1);
          } catch(InvalidMidiDataException imde) {
           System.out.println(imde.getMessage());
           System.exit(0);
          }
         }
         break;
        case 2: // amplitude
         sensorMax=Math.max(sensorMax,sensorDiff);
         if(i==numSensors-1) {
          try {
           ShortMessage sm=new ShortMessage();
           sm.setMessage(ShortMessage.NOTE_ON, 0, gui.getNote()+10, (int)Math.min(127,param*sensorMax*127.0f));
           synthRcvr.send(sm, -1);
          } catch(InvalidMidiDataException imde) {
           System.out.println(imde.getMessage());
           System.exit(0);
          }
          sensorMax=0.0f;
         }
         break;
        case 3: // frequency
         sensorMax=Math.max(sensorMax,sensorDiff);
         if(i==numSensors-1) {
          try {
           int note=(int)Math.min(127,param*sensorMax*127.0f);
           gui.setNote(note);
           ShortMessage sm=new ShortMessage();
           sm.setMessage(ShortMessage.NOTE_ON, 0, note, 90);
           synthRcvr.send(sm, -1);
          } catch(InvalidMidiDataException imde) {
           System.out.println(imde.getMessage());
           System.exit(0);
           }
          sensorMax=0.0f;
         }
         break;
       }
      }
     } else {
      synth.getChannels()[0].allNotesOff();
     }

    }
    input="";
   }
  }
 }

 private void updateOldSensorValues() {
  for(int i=0; i<oldSensorValues.length; i++) oldSensorValues[i][0]=oldSensorValues[i][1];
 }

}