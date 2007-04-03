#include<iostream>
#include<vector>
#include<stdlib.h>
using namespace std; 

#include <selforg/multilayerffnn.h>
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
  cout << "first Weightmatrix " << endl <<  net.weights[0] << endl;
  
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



int main(){
  srand(time(0));
  
  vector<Layer> layers1;
  layers1.push_back(Layer(2,0));
  MultiLayerFFNN netlinear(0.1, layers1);
  netlinear.init(2,2);

  vector<Layer> layers2;
  layers2.push_back(Layer(3, 0.1, FeedForwardNN::sigmoid));
  layers2.push_back(Layer(2));
  MultiLayerFFNN netnonlinear(0.1, layers2);
  netnonlinear.init(2,2);

  vector<Layer> layers3;
  layers3.push_back(Layer(3, 0.1, FeedForwardNN::sigmoid));
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
  return 0;
}


