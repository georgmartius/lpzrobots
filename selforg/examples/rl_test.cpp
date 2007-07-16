#include<iostream>
#include<vector>
#include<stdlib.h>
using namespace std; 

#include <selforg/qlearning.h>
using namespace matrix;


void testQLearning(){
  QLearning q(0.1,0.9,0.05, 1);
  q.init(5,2);
  cout << "Q Learning 5x2\n";
    
  int x=0;
  for(int i=0; i < 100; i++){
    int a = q.select(x);
    if(a==0) x--;
    if(a==1) x++;
    if(x<0) x=0;
    if(x>4) x=4;
    double r = (x==4) ? 1 : 0;
    q.learn(x,a,r);        
    if(x==4) x=0;
  }

  cerr << q.getQ();
  fprintf(stderr,"\nReward: %lf\n", q.collectedReward);

}




int main(){
  srand(time(0));
  
   cout << "******************** TEST Q-Learning\n";
  testQLearning();
 
  return 0;
}


