import javax.swing.*;
import java.awt.event.*;
import java.text.*;

public class SoundManGUI extends JFrame {
 private float param;
 private int mode;
 private int numSensors;
 private int curInstrument;
 private JComboBox instruCB;
 private JLabel l3;

 public SoundManGUI(float p, int m) {
  param=p;
  mode=m;
  getContentPane().setLayout(null);
  setTitle("SoundMan");
  setResizable(false);
  setSize(250,160);

  ////////// mode //////////
  JRadioButton discRB=new JRadioButton("Discrete");
  discRB.setBounds(5, 10, 75, 20);
  getContentPane().add(discRB);

  JRadioButton amplRB=new JRadioButton("Amplitude");
  amplRB.setBounds(80, 10, 80, 20);
  getContentPane().add(amplRB);

  JRadioButton freqRB=new JRadioButton("Frequency");
  freqRB.setBounds(160, 10, 80, 20);
  getContentPane().add(freqRB);

  ButtonGroup modeBG=new ButtonGroup();
  modeBG.add(discRB);
  modeBG.add(amplRB);
  modeBG.add(freqRB);

  if(mode==1) discRB.setSelected(true);
  else if(mode==2) amplRB.setSelected(true);
  else freqRB.setSelected(true);

  ////////// param //////////
  JLabel l1=new JLabel("Parameter:");
  l1.setBounds(10, 40, 80, 20);
  getContentPane().add(l1);

  DecimalFormat df=new DecimalFormat();
  df.setMaximumFractionDigits(2);
  df.setMinimumFractionDigits(0);
  final JFormattedTextField paramFTF=new JFormattedTextField(df);
  paramFTF.setText(NumberFormat.getInstance().format(param));
  paramFTF.setBounds(85, 40, 50, 20);
  getContentPane().add(paramFTF);

  JButton paramB=new JButton("Accept");
  paramB.setBounds(150, 40, 75, 20);
  getContentPane().add(paramB);
  paramB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     param=new Float(paramFTF.getText().replace(',','.')).floatValue();
    }
   }
  );

  ////////// instruments //////////
  JLabel l2=new JLabel("Instrument:");
  l2.setBounds(10, 70, 80, 20);
  getContentPane().add(l2);

  instruCB=new JComboBox();
  instruCB.setBounds(90, 70, 140, 20);
  getContentPane().add(instruCB);

  ////////// sensors //////////
  l3=new JLabel("Number of sensors: -");
  l3.setBounds(10, 100, 200, 20);
  getContentPane().add(l3);
 }

 public void setNumSensors(int n) {
  numSensors=n;
  l3.setText("Number of sensors: "+n);
 }

 public float getParam() {
  return param;
 }

 public int getMode() {
  return mode;
 }

 public boolean instrumentChanged() {
  if(curInstrument!=instruCB.getSelectedIndex()) {
   curInstrument=instruCB.getSelectedIndex();
   return true;
  }
  return false;
 }

 public int getInstrument() {
  return instruCB.getSelectedIndex();
 }

 public void setInstruments(String[] s) {
  for(int i=0; i<s.length; i++) instruCB.addItem(s[i]);
 }

}
