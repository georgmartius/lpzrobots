#include<iostream>
#include<vector>
#include<stdlib.h>
using namespace std; 

#include <selforg/multilayerffnn.h>
#include <selforg/controllernet.h>
#include <selforg/som.h>
#include <selforg/neuralgas.h>
#include <selforg/matrix.h>
using namespace matrix;

void test2x2(MultiLayerFFNN& net){
  double data1[2] = {0,1};
  Matrix input1(2,1,data1);
  double data2[2] = {0,0.9};
  Matrix output1(2,1,data2);
  double data3[2] = {1,0};
  Matrix input2(2,1,data3);
  double data4[2] = {0.4,0.6};
  Matrix output2(2,1,data4);
    
  // train
  for(int i=0; i < 500; i++){
    net.process(input1);
    net.learn(input1, output1);        
    net.process(input2);
    net.learn(input2, output2);        
  }

  // test
  cout << "TEST" << endl;
  cout << (net.process(input1)^T) << " should be around " << (output1^T) << endl; 
  cout << (net.process(input2)^T) << " should be around " << (output2^T) << endl; 
}

MultiLayerFFNN testnonlinear(){
  double o0[1] = {0.1};
  double o1[1] = {0.9};

  double i0[2] = {0 , 0};
  double i1[2] = {1 , -1};
  double i2[2] = {-1, 0};
  double i3[2] = {-1, 1};
  double i4[2] = { 1, 1};


  vector<Layer> layers;
  layers.push_back(Layer(3, 0.5 , FeedForwardNN::tanh));
  layers.push_back(Layer(2));
  MultiLayerFFNN net(0.05, layers);
  net.init(2,1);
    
  // train
  for(int i=0; i < 500; i++){
    net.process(Matrix(2, 1, i0));
    net.learn(Matrix(2, 1, i0), Matrix(1, 1, o0));        
    net.process(Matrix(2, 1, i1));
    net.learn(Matrix(2, 1, i1), Matrix(1, 1, o0));        
    net.process(Matrix(2, 1, i2));
    net.learn(Matrix(2, 1, i2), Matrix(1, 1, o0));        
    net.process(Matrix(2, 1, i3));
    net.learn(Matrix(2, 1, i3), Matrix(1, 1, o0));
    net.process(Matrix(2, 1, i4));
    net.learn(Matrix(2, 1, i4), Matrix(1, 1, o1));
  }

  // test
  cout << "TEST" << endl;
  cout << net.process(Matrix(2, 1, i0))  << " should be around " << o0[0] << endl; 
  cout << net.process(Matrix(2, 1, i1))  << " should be around " << o0[0] << endl; 
  cout << net.process(Matrix(2, 1, i2))  << " should be around " << o0[0] << endl; 
  cout << net.process(Matrix(2, 1, i3))  << " should be around " << o0[0] << endl; 
  cout << net.process(Matrix(2, 1, i4))  << " should be around " << o1[0] << endl; 
  return net;
}

void testresponse(MultiLayerFFNN net){
  double i0[2] = {0,1};
  Matrix input (2, 1, i0);
  net.process(input);
  Matrix J = net.response(input);
  cout << "Responsematrix for " << (input^T) << endl << endl << J << endl;
  cout << "first Weightmatrix " << endl <<  net.getWeights(0) << endl;
  
  Matrix o = J*input;
  cout << "Test: " << (o^T)  << " should be around " << (net.process(input)^T)<< endl;
}

void testinvertation(MultiLayerFFNN net){
  double i0[2] = {0,  1.0};
  double xsi_[2] = {0.1,0};
  Matrix input (2, 1, i0);
  Matrix xsi(2,1,xsi_);
  Matrix o = net.process(input);
  Matrix eta = net.inversion(input, xsi);
  cout << "Normal output: " << (o^T) << endl;
  cout << "inversion for " << (input^T) << " and xsi " << (xsi^T)<< endl << ":" << (eta^T) << endl;
  o = net.process(input+eta);
  cout << "Shifted Output after inversion: " << (o^T) << endl;
}




void testsom1D(){
  SOM net(1, 5, 0.1,1,"SOM", "01");
  net.init(2,100);
  cout << "SOM 1D 2-100 (5)\n";
  FOREACHC(SOM::Neighbourhood, net.getNeighbourhood(), i){
    cout << i->first << "\t w: " << i->second << "\n";
  }
  Matrix inp(2,1);
  Matrix out(100,1);
    
  for(int i=0; i < 10000; i++){
    inp.toMap(random_minusone_to_one);
    net.process(inp);
    net.learn(inp,out);
  }
  FILE* f = fopen("som1D.weights","w");
  net.printWeights(f);
  fclose(f);
}

void testsom2D(){
  SOM net(2, 3, 0.01,1,"SOM", "01");
  net.init(2,36,0);
  cout << "SOM 2D 2-36 (3)\n";
  FOREACHC(SOM::Neighbourhood, net.getNeighbourhood(), i){
    cout << (i->first^T) << "\t w: " << i->second << "\n";
  }
  Matrix inp(2,1);
  Matrix out(100,1);
    
  for(int i=0; i < 10000; i++){
    inp.toMap(random_minusone_to_one);
    net.process(inp);
    net.learn(inp,out);
  }
  net.setParam("eps",0.001);
  for(int i=0; i < 5000; i++){
    inp = inp.map(random_minusone_to_one) * 0.05;
    net.process(inp);
    net.learn(inp,out);
  }
  Matrix offset(2,1);
  offset.val(0,0)=0.5;
  offset.val(1,0)=-0.8;
  for(int i=0; i < 5000; i++){
    inp = inp.map(random_minusone_to_one) * 0.05 + offset;
    net.process(inp);
    net.learn(inp,out);
  }

  FILE* f = fopen("som2D.weights","w");
  net.printWeights(f);
  fclose(f);
}


void testsom1D_local(){
  SOM net(1,3, 0.01,1,"SOM", "01");
  net.init(2,36,1.5);
  cout << "SOM 1D 2-36 (3)\n";
  FOREACHC(SOM::Neighbourhood, net.getNeighbourhood(), i){
    cout << i->first << "\t w: " << i->second << "\n";
  }
  Matrix inp(2,1);
  Matrix out(100,1);
    
  for(int i=0; i < 100; i++){
    inp = inp.map(random_minusone_to_one) * 0.1;
    net.process(inp);
    net.learn(inp,out);
  }

  FILE* f = fopen("som1Dlocal.weights","w");
  net.printWeights(f);
  fclose(f);

}

void testneuralgas(){
  NeuralGas net(2, 0.01,0);
  net.init(2,36,0);
  cout << "NeuralGas 2D 2-36\n";

  Matrix inp(2,1);
  Matrix out(100,1);
    
  for(int i=0; i < 10000; i++){
    inp.toMap(random_minusone_to_one);
    net.process(inp);
    net.learn(inp,out);
  }
  net.printCellsizes(stdout);
  cout << "\n";
  net.setParam("eps",0.001);
  for(int i=0; i < 5000; i++){
    inp = inp.map(random_minusone_to_one) * 0.05;
    net.process(inp);
    net.learn(inp,out);
  }
  Matrix offset(2,1);
  offset.val(0,0)=0.5;
  offset.val(1,0)=-0.8;
  for(int i=0; i < 5000; i++){
    inp = inp.map(random_minusone_to_one) * 0.05 + offset;
    net.process(inp);
    net.learn(inp,out);
  }
  net.printCellsizes(stdout);
  cout << "\n";
  FILE* f = fopen("neural_gas.weights","w");
  net.printWeights(f);
  fclose(f);
}

void testprojections(ControllerNet net){
  double i0[2] = {0,  1.0};
  Matrix input (2, 1, i0);
  double xsi_[2] = {-.05,.1};
  Matrix xsi(2,1,xsi_);
  Matrix o = net.process(input);
  cout << "Normal input/ output: " << (input^T) << " / " << (o^T) << endl;
  for(unsigned int i=0; i<net.getLayerNum();i++){
    cout << "layer " << i << ": " << endl << net.getWeights(i) << endl;
    cout << " output: " << (net.getLayerOutput(i)^T) << endl;
  }
  Matrices zeta1;
  Matrix xsi_fw   = net.forwardpropagation(xsi, 0, &zeta1);
  Matrices zeta2;
  Matrix xsi_rec = net.backprojection(xsi_fw, 0, &zeta2);  
  cout << "ForwardProp -> BackProj = ID" << endl;
  cout << "Shift: " << (xsi^T) << endl;
  cout << "fw propagation: " << (xsi_fw^T) <<  endl; 
  FOREACHC(Matrices, zeta1, z){
    cout << " | zeta " << ((*z)^T);
  }

  cout << endl << " and backprojection " << (xsi_rec^T) << endl;
  FOREACHC(Matrices, zeta2, z){
    cout << " | zeta " << ((*z)^T);
  } 
  cout << endl << "==>Difference (should be zero) :" << ((xsi - xsi_rec)^T) << endl;
  Matrix o2 = net.process(input+xsi);
  cout << "forward test: input+xsi/ output': " << ((input+xsi)^T) << " / " << (o2^T) << endl;
  cout << "--- Difference: " << ((o+xsi_fw - o2)^T) << endl;


  xsi_fw   = net.forwardprojection(xsi, 0, &zeta1);
  xsi_rec = net.backpropagation(xsi_fw, 0, &zeta2);  
  cout << "ForwardProj -> BackProp = ID" << endl;
  cout << "Shift: " << (xsi^T) << endl;
  cout << "fw projection: " << (xsi_fw^T) <<  endl; 
  FOREACHC(Matrices, zeta1, z)
    cout << " | zeta " << ((*z)^T);
  cout << endl << " and backpropagation " << (xsi_rec^T) << endl;
  FOREACHC(Matrices, zeta2, z){
    cout << " | zeta " << ((*z)^T);
  } 
  cout << endl << "==>Difference (should be zero) :" << ((xsi - xsi_rec)^T) << endl;

}


int controllernettest(){
  vector<Layer> layers1;
  layers1.push_back(Layer(2,0));
  ControllerNet netlinear(layers1);
  netlinear.init(2,2,0,1);

  vector<Layer> layers2;
  layers2.push_back(Layer(2, 0.1, FeedForwardNN::tanhr));
  layers2.push_back(Layer(2));
  ControllerNet netnonlinear(layers2);
  netnonlinear.init(2,2,0,1);

  vector<Layer> layers3;
  layers3.push_back(Layer(2, 0.1, FeedForwardNN::linear));
  layers3.push_back(Layer(2));
  ControllerNet netbypass(layers3, true);
  netbypass.init(2,2,0,1);

  cout << "************* C O N T R O L L E R N E T ***********\n";

  netlinear.setParam("lambda",   0.00);
  netnonlinear.setParam("lambda",0.00);
  netbypass.setParam("lambda",   0.00);
  cout << "******************** testprojections (linear)\n";
  testprojections(netlinear);
  cout << "******************** testprojections (nonlinear)\n";
  testprojections(netnonlinear);
  cout << "******************** testprojections (bypass)\n"; 
  testprojections(netbypass);
  return 1;
}


int main(){
  srand(time(0));
  bool mltests = false;
  bool somtests = false;
  bool contrtests = true;

  if(mltests){
    vector<Layer> layers1;
    layers1.push_back(Layer(2,0));
    MultiLayerFFNN netlinear(0.1, layers1);
    netlinear.init(2,2);
    
    vector<Layer> layers2;
    layers2.push_back(Layer(2, 0.1, FeedForwardNN::tanhr));
    layers2.push_back(Layer(2));
    MultiLayerFFNN netnonlinear(0.1, layers2);
    netnonlinear.init(2,2);
    
    vector<Layer> layers3;
    layers3.push_back(Layer(2, 0.1, FeedForwardNN::tanhr));
    layers3.push_back(Layer(2));
    MultiLayerFFNN netbypass(0.1, layers3, true);
    netbypass.init(2,2);

    cout << "******************** TEST 2x2 linear\n";
    test2x2(netlinear);
    cout << "******************** TEST 2x2 nonlinear\n";
    test2x2(netnonlinear);
    cout << "******************** TEST 2x2 bypass\n";
    test2x2(netbypass);
    cout << "******************** testnonlinear\n";
    //const MultiLayerFFNN& net = testnonlinear();
    testnonlinear();
    cout << "******************** testresponse (linear)\n";
    testresponse(netlinear);
    cout << "******************** testresponse (nonlinear)\n";
    testresponse(netnonlinear);
    cout << "******************** testresponse (bypass)\n";
    testresponse(netbypass);
    cout << "******************** testinvertation (linear)\n";
    testinvertation(netlinear);
    cout << "******************** testinvertation (nonlinear)\n";
    testinvertation(netnonlinear);
    cout << "******************** testinvertation (bypass)\n";
    testinvertation(netbypass);
  }
  if(somtests){
    cout << "******************** TEST SOM\n";
    testsom1D();
    testsom2D();
    testsom1D_local();
    
    cout << "******************** TEST Neural Gas\n";
    testneuralgas();
  }
  if(contrtests){
    controllernettest();
  }

 
  return 0;
}


