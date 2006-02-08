#include<iostream>
#include<vector>
using namespace std; 

#include "selforg/multilayerffnn.h" 
#include "matrix.h" 
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
  cout << net.process(input1);      
  cout << net.process(input2);    
}

void testnonlinear(){
  double o0[1] = {0};
  double o1[1] = {1};

  double i0[2] = {0,0};
  double i1[2] = {0,1};
  double i2[2] = {-1,0};
  double i3[2] = {-1,1};
  double i4[2] = {1,1};

  vector<Layer> layers;
  layers.push_back(Layer(1, 0.1, FeedForwardNN::sigmoid, FeedForwardNN::dsigmoid));
  layers.push_back(Layer(2));
  MultiLayerFFNN net(0.1, layers);
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
  cout << net.process(Matrix(2, 1, i0));      
  cout << net.process(Matrix(2, 1, i1));      
  cout << net.process(Matrix(2, 1, i2));      
  cout << net.process(Matrix(2, 1, i3));      
  cout << net.process(Matrix(2, 1, i4));      

}

int main(){
  test2x2();
  testnonlinear();
  return 0;
}


