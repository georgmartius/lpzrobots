
#include <string>
#include <vector>
#include "Socket.h"

using namespace std;


int main(int argc, char** argv){
  Socket s;
  s.accept(4000);
  // for(int i=-5000; i<5000; i+=432){
  //   cout << i << endl;
  //   s << i;
  // }
  // double d;
  // for( d=-5000; d<5000; d+=123.123){
  //   cout << d << endl;
  //   s << d;
  // }

  vector<int> vi;
  vi.push_back(10);
  vi.push_back(4);
  vi.push_back(5);
  vi.push_back(11);
  cout << 10 << " " << 4 <<  " " << 5 <<  " "  << 11 << endl;
  s << vi;

  vector<double> vd;
  vd.push_back(10.1213);
  vd.push_back(4.1234);
  vd.push_back(5.64);
  vd.push_back(11.12);
  s << vd;
  
  s << string("Huhu from the other side.");

  s.close();
  return 0;
}
