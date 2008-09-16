#include<iostream>
#include<vector>
#include<stdlib.h>
using namespace std; 

#include <selforg/qlearning.h>
using namespace matrix;


void testQLearning(){
  int size=10;
  //  QLearning q(0.2,0.9,0.05, 5,false,false);
  QLearning q(0.2,0.9,0.05, 5,false,true);
  q.init(size,2);
  cout << "Q Learning 10x2\n";
  cerr << q.getQ();
    
  int x=0;
  int a = 1;
  for(int i=0; i < 10000; i++){
    if(a==0) x--;
    if(a==1) x++;
    if(x<0) x=0;
    if(x>=size) x=size-1;
    printf("action: %i state: %i\n",a, x);
    double r = (x==size-1) ? 1 : 0;
    a = q.select(x);
    q.learn(x,a,r);
    if(x==size-1) {
      x=0;
      q.learn(x,a,r);
      q.reset();
      
    }
  }

//   for(int i=0; i < 6; i++){
//     int a = q.select(x);
//     if(a==0) x--;
//     if(a==1) x++;    
//     if(x<0) x=0;
//     if(x>4) x=4;
//     x=i%5;
//     printf("action: %i state: %i\n",a, x);
//     double r = (x==4) ? 1 : 0;
//     q.learn(x,a,r);        
//     if(x==4) x=0;
//   }

  cerr << q.getQ();
  fprintf(stderr,"\nReward: %lf\n", q.getCollectedReward());

}




int main(){
  srand(time(0));
  
  cout << "******************** TEST Q-Learning\n";
  testQLearning();
 
  return 0;
}


