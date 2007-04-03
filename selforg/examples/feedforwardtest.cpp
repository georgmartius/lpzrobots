#include<iostream>
#include<vector>
#include<stdlib.h>
using namespace std; 

#include <selforg/multilayerffnn.h>
#include <selforg/matrix.h>
using namespace matrix;

void test2x2(){
  double data1[2] = {0,1};
  Matrix input1(2,1,data1);
  double data2[2] = {0,0.9};
  Matrix output1(2,1,data2);
  double data3[2] = {1,0};
  Matrix input2(2,1,data3);
  double data4[2] = {0.4,0.6};
  Matrix output2(2,1,data4);

  vector<Layer> layers;
  layers.push_back(Layer(3, 0.1, FeedForwardNN::sigmoid, FeedForwardNN::dsigmoid));
  layers.push_back(Layer(2));
  MultiLayerFFNN net(0.1, layers);
  net.init(2,2);
    
  // train
  for(int i=0; i < 500; i++){
    net.learn(input1, output1);        
    net.learn(input2, output2);        
  }

  // test
  cout << "TEST" << endl;
  cout << net.process(input1) << endl;;      
  cout << net.process(input2) << endl;;    
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
  layers.push_back(Layer(3, 0.5 , FeedForwardNN::tanh,FeedForwardNN::dtanh));
  layers.push_back(Layer(2));
  MultiLayerFFNN net(0.05, layers);
  net.init(2,1);
    
  // train
  for(int i=0; i < 500; i++){
    net.learn(Matrix(2, 1, i0), Matrix(1, 1, o0));        
    net.learn(Matrix(2, 1, i1), Matrix(1, 1, o0));        
    net.learn(Matrix(2, 1, i2), Matrix(1, 1, o0));        
    net.learn(Matrix(2, 1, i3), Matrix(1, 1, o0));
    net.learn(Matrix(2, 1, i4), Matrix(1, 1, o1));
  }

  // test
  cout << "TEST" << endl;
  cout << net.process(Matrix(2, 1, i0)) << endl;;      
  cout << net.process(Matrix(2, 1, i1)) << endl;;      
  cout << net.process(Matrix(2, 1, i2)) << endl;;      
  cout << net.process(Matrix(2, 1, i3)) << endl;;      
  cout << net.process(Matrix(2, 1, i4)) << endl;;      
  return net;
}

void testinvertation(const MultiLayerFFNN& net){
  double i0[2] = {1,1};
  Matrix input (2, 1, i0);
  Matrix J = net.response(input);
  cout << "Responsematrix for " << (input^T) << endl << endl << J << endl;
  Matrix o = J*input;
  cout << "Test: " << o << endl;
}

void testinvertation2(){
  std::vector<Layer> layers;
  layers.push_back(Layer(2));
  layers.push_back(Layer(2));
  MultiLayerFFNN net(0.1, layers);
  net.init(2,2,1.0);

  double i0[2] = {1,1};
  Matrix input (2, 1, i0);
  Matrix J = net.response(input);
  cout << "Responsematrix for " << (input^T) << endl << endl  << J << endl;
  Matrix o = J*input;
  cout << "Test: " << o << endl;

}

void testinvertation3(const MultiLayerFFNN& net){
  double i0[2] = {1,  1};
  double xsi_[2] = {0.1};
  Matrix input (2, 1, i0);
  Matrix xsi(1,1,xsi_);
  MultiLayerFFNN net2=net;
  Matrix eta = net2.inversion(input, xsi);
  Matrix o = net2.process(input);
  cout << "Normal output: " << o << endl;
  cout << "inversion for " << (input^T) <<endl << ":" << eta << endl;
  o = net2.process(input+eta);
  cout << "Shifted Output after inversion: " << (o^T) << endl;
}

void testinvertation4(){
  std::vector<Layer> layers;
  layers.push_back(Layer(2));
  layers.push_back(Layer(2));
  MultiLayerFFNN net(0.1, layers);
  net.init(2,2,1.0);

  double i0[2] = {1,1};
  double i1[2] = {0.1,-0.2};
  Matrix input (2, 1, i0);
  Matrix xsi(2,1,i1);
  Matrix eta = net.inversion(input, xsi);
  Matrix o = net.process(input);
  cout << "Normal output: " << (o^T) << endl;
  cout << "inversion for " << (input^T) <<endl << ":" << (eta^T) << endl;
  o = net.process(input+eta);
  cout << "Shifted Output after inversion: " << (o^T) << endl;


}



int main(){
  srand(time(0));
  cout << "******************** TEST 2x2\n";
  test2x2();
  cout << "******************** testnonlinear\n";
  const MultiLayerFFNN& net = testnonlinear();
  cout << "******************** testinvertation\n";
  testinvertation(net);
  cout << "******************** testinvertation2\n";
  testinvertation2();
  cout << "******************** testinvertation3\n";
  testinvertation3(net);
  cout << "******************** testinvertation4\n";
  testinvertation4();
  return 0;
}


