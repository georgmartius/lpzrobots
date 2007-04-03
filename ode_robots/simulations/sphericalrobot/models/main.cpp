#include <assert.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <selforg/multilayerffnn.h>
#include <selforg/elman.h>
#include "datafunc.h"

using namespace std;
using namespace matrix;

bool check4Number(const char* c){
  const char* p = c;
  while(*p != 0){
    if(*p >= '0' && *p <= '9') return true;
    p++;
  }
  return false;
}

bool parseDataFile(vector<Matrix>& data, FILE* f){
  char buffer[1024];  
  int i;
  double dat[1024];
  Matrix m;
  while(fgets(buffer, 1024, f)){    
    if(buffer[0]=='#') continue;    
    i=0;
    char* p;
    p=strtok(buffer," ");
    if(!p) return false;
    dat[i] = atof(p);    
    i++;
    while((p=strtok(NULL," "))!=NULL )  {
      if(!check4Number(p)) continue;
      dat[i] = atof(p);
      i++;
    };
    m.set(i,1,dat);
    data.push_back(m);
  };
  return true;
}

// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}



int main(int argc, char** argv){
  char* filename = "data";
  int index;
  bool training = true;
  const char* networkpath ="net";
  string outnetworkpath ="net.net";
  ofstream dumpfile;
  bool loadnetwork=false;
  int iterations = 1;
  double eps = 0.01;
  unsigned int maxhistory=20;
  int numHidden=3;
  DataFunc inp = &tm123;
  DataFunc out = &t;
  FILE* f;  
  bool useElman=false;
  bool useJordan=false;
  bool useBypass=false;

  /// PARSE Parameter
  index = contains(argv,argc,"-type");
  if(index!=0 && argc > index) {    
    char* type = argv[index];
    useElman= (strchr(type,'e')!=0);
    useJordan= (strchr(type,'j')!=0);
    useBypass= (strchr(type,'b')!=0);
    cout << "# network type: " << (!useElman && !useJordan ? "Feedforward" : "" )
	 << (useElman ? "Elman " : "") << (useJordan ? "Jordan " : "")  
	 << (useBypass ? "with Bypass" : "") << endl;  
  }
  index = contains(argv,argc,"-f");
  if(index!=0 && argc > index) {    
    filename = argv[index];
    cout << "# data file: " << filename << endl;  
  }
  index = contains(argv,argc,"-n");
  if(index!=0 && argc > index) {
    networkpath = argv[index];
    outnetworkpath = string(networkpath) + ".net";   
    loadnetwork = true;
    cout << "# load network: " << networkpath << endl;  
  }
  index = contains(argv,argc,"-o");
  if(index!=0 && argc > index) {
    outnetworkpath = argv[index];
    cout << "# store network to: " << outnetworkpath << endl;  
  }
  index = contains(argv,argc,"-d");
  if(index!=0 && argc > index) {
    dumpfile.open(argv[index]);
    if(!dumpfile.is_open()) perror("could not open dumpfile for writing");
    else { 
      dumpfile << "#C Results[0] Results[1] Data[0] Data[1]" << endl;
      cout << "# dump output to: " << argv[index] << endl;  
    }
   
  }
  index = contains(argv,argc,"-i");
  if(index!=0 && argc > index) { iterations = atoi(argv[index]); }
  index = contains(argv,argc,"-u");
  if(index!=0 && argc > index) { numHidden = atoi(argv[index]); }
  index = contains(argv,argc,"-inp");
  if(index!=0 && argc > index) { inp = datafunctions(argv[index]); }
  index = contains(argv,argc,"-out");
  if(index!=0 && argc > index) { out = datafunctions(argv[index]); }
  if(contains(argv,argc,"-t")){
    cout << "# test phase network" << endl;    
    training=false;
  }
  index = contains(argv,argc,"-e");
  if(index!=0 && argc > index) {
    eps = atof(argv[index]);
  }
  if(training) cout << "# EPS: " << eps << endl; 
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-f file] [-n network] [-o newnet] [-t] [-d output] [-i num] [-e eps] [-inp dfunc] [-out dfunc]\n",argv[0]);
    printf("\t-f file\tuse this data file (def: data)\n");
    printf("\t-type f|{e|j}*[b]\ttype of network: feedforward, elman, jordan, or both, with,w/o bypass\n");
    printf("\t-n network\tfile to load network from (def: create a new one)\n");
    printf("\t-o newnet\tfile to store new network to (def: network.net)\n");
    printf("\t-t   \ttest mode (requires -n)\n");
    printf("\t-d output\tfile to dump results of training (def: none)\n");
    printf("\t-i num\tnumber of iterations for training (def: 1)\n");
    printf("\t-u num\tnumber of hidden units (def: 3)\n");
    printf("\t-e eps\tlearning rate (def: 0.01)\n");
    printf("\t-inp dfunc\tinput data function (def: tm123)\n");
    printf("\t-out dfunc\toutput data function (def: t)\n");
    exit(0);
  }
  
  /// LOAD FILE
  f = fopen(filename, "r");
  assert(f);
  vector<Matrix> data;
  if(!parseDataFile(data,f)) {
    fprintf(stderr, "Cannot parse file");
  }
  fclose(f);
  
  unsigned int inputdim = inp(data,maxhistory).getM();
  unsigned int outputdim = out(data,maxhistory).getM();

  /// LOAD/INITALISE NETWORK
  MultiLayerFFNN* net;

  if(loadnetwork){
    if(useElman || useJordan) 
      net = new Elman(eps, vector<Layer>(), false, false); // it is loaded anyway
    else
      net = new MultiLayerFFNN(eps, vector<Layer>());
    f = fopen(networkpath, "r");
    assert(f);
    if(!net->restore(f)){
      cerr << "could not read network: " << networkpath << endl;
      exit(1);
    }
    fclose(f);
    if(net->getInputDim() != inputdim || net->getOutputDim() != outputdim){
      cerr << "loaded networks dimension do not fit: observed: " 
	   << net->getInputDim() << "," << net->getOutputDim()
	   << " exected: " << inputdim << "," << outputdim << endl;
      exit(1);
    }
  }else{
    vector<Layer> layers;
    //layers.push_back(Layer(2, 1.0, FeedForwardNN::sigmoid, FeedForwardNN::dsigmoid));
    layers.push_back(Layer(numHidden, 1.0 , FeedForwardNN::tanh));
    layers.push_back(Layer(1));
    if(useElman || useJordan)
      net = new Elman(eps, layers, useElman, useJordan, useBypass);
    else
      net = new MultiLayerFFNN(eps, layers, useBypass);
    net->init(inputdim,outputdim);
    cerr << "Dimensions: " << inputdim << "," << outputdim << endl;
  }  
  cout << "# Number of Hidden units: " << numHidden <<  endl;
  
  /// TRAIN AND/OR CHECK
  cout << "#C error activity avgerror" << endl;
  double slidingavg=0.5;
  for(int k=0; k<iterations; k++){
    for(unsigned int i=maxhistory; i<data.size()-1; i++){    
      const Matrix& sensors = inp(data, i);
      const Matrix& result  = net->process(sensors); // activate with next data in order judge prediction
      const Matrix& nomout = out(data,i);
      double diff = sqrt((nomout - result).multTM().val(0,0));          
      slidingavg = slidingavg*0.999 + diff*0.001;
      double datdiff = (out(data,i+1) - nomout).multTM().val(0,0) + 0.05;    
      cout << diff << " " << datdiff << " " << slidingavg  << endl;      
      if(dumpfile.is_open()) dumpfile << (result^T) << (nomout^T) << endl;
      if(training){
	net->learn(sensors, out(data,i)); // learn
      }
    }
  }
  
  if(training){
    f = fopen((outnetworkpath).c_str(), "w");
    assert(f);
    net->store(f);
    fclose(f);
  }
  if(dumpfile.is_open()) dumpfile.close();
  return 0;
}


  //  FOREACHC (vector<Matrix>, data, it){
  //    cout << ((*it)^T);
  //  }
