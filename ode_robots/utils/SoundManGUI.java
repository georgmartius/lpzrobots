import javax.swing.*;
import java.awt.event.*;
import java.text.*;

public class SoundManGUI extends JFrame {
 private float param;
 private int curInstrument;
 private JComboBox instruCB;

 public SoundManGUI(float p) {
  param=p;
  getContentPane().setLayout(null);
  setTitle("SoundMan");
  setResizable(false);
  setSize(250,100);

  ////////// param //////////
  JLabel l1=new JLabel();
  l1.setText("Parameter:");
  l1.setBounds(10, 10, 80, 20);
  getContentPane().add(l1);

  DecimalFormat df=new DecimalFormat();
  df.setMaximumFractionDigits(2);
  df.setMinimumFractionDigits(0);
  final JFormattedTextField paramFTF=new JFormattedTextField(df);
  paramFTF.setText(NumberFormat.getInstance().format(param));
  paramFTF.setBounds(85, 10, 50, 20);
  getContentPane().add(paramFTF);

  JButton paramB=new JButton("Accept");
  paramB.setBounds(150, 10, 75, 20);
  getContentPane().add(paramB);
  paramB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     param=new Float(paramFTF.getText().replace(',','.')).floatValue();
    }
   }
  );

  ////////// instruments //////////
  JLabel l2=new JLabel();
  l2.setText("Instrument:");
  l2.setBounds(10, 40, 80, 20);
  getContentPane().add(l2);

  instruCB=new JComboBox();
  instruCB.setBounds(90, 40, 140, 20);
  getContentPane().add(instruCB);

 }

 public float getParam() {
  return param;
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
