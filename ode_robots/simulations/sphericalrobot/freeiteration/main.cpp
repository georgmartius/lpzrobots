#include <assert.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <selforg/multilayerffnn.h>
#include <selforg/datafunc.h>

#define FOREACH(colltype, coll, it) for( colltype::iterator it = (coll).begin(); it!= (coll).end(); it++ )
#define FOREACHC(colltype, coll, it) for( colltype::const_iterator it = (coll).begin(); it!= (coll).end(); it++ )

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

void writeVecElemNames(ostream& str, const string& name, const Matrix& m){
  for(int i=0; i < m.getM(); i++){
    str << name << '[' << i << ']' << ' ';
  }
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
  const char* networkpath ="net";
  ofstream dumpfile;
  bool loadnetwork=false;
  int iterations = 100;
  int skip = 1000;
  double eps = 0.01;
  int maxhistory=20;
  DataFunc inp = &tm123;
  DataFunc out = &t;
  FILE* f;

  /// PARSE Parameter
  index = contains(argv,argc,"-f");
  if(index!=0 && argc > index) {
    filename = argv[index];
    cout << "# data file: " << filename << endl;
  }
  index = contains(argv,argc,"-n");
  if(index!=0 && argc > index) {
    networkpath = argv[index];
    loadnetwork = true;
    cout << "# load network: " << networkpath << endl;
  }
  index = contains(argv,argc,"-i");
  if(index!=0 && argc > index) {
    iterations = atoi(argv[index]);
  }
  index = contains(argv,argc,"-s");
  if(index!=0 && argc > index) {
    skip = atoi(argv[index]);
  }
  index = contains(argv,argc,"-inp");
  if(index!=0 && argc > index) {
    inp = datafunctions(argv[index]);
  }
  index = contains(argv,argc,"-out");
  if(index!=0 && argc > index) {
    out = datafunctions(argv[index]);
  }
  if(contains(argv,argc,"-h")!=0 || !loadnetwork) {
    printf("Usage: %s [-f file] [-n network]  [-i num] [-s start] [-inp dfunc] [-out dfunc]\n",argv[0]);
    printf("\t-f file\tuse this data file (def: data)\n");
    printf("\t-n network\tfile to load network from (def: create a new one)\n");
    printf("\t-i num\tnumber of data steps to predict (def: 100)\n");
    printf("\t-s skip\ttime step to skip (def: 1000)\n");
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
  MultiLayerFFNN net(eps, vector<Layer>());

  f = fopen(networkpath, "r");
  assert(f);
  if(!net.restore(f)){
    cerr << "could not read network: " << networkpath << endl;
    exit(1);
  }
  fclose(f);
  if(net.getInputDim() != inputdim || net.getOutputDim() != outputdim){
    cerr << "loaded networks dimension do not fit: observed: "
         << net.getInputDim() << "," << net.getOutputDim()
         << " exected: " << inputdim << "," << outputdim << endl;
    exit(1);
  }

  // prediction dataset
  vector<Matrix> preddata;

  /// FREE Iteration
  cout << "#C ";
  writeVecElemNames(cout, "Pred",  out(data,maxhistory));
  writeVecElemNames(cout, "Real",  out(data,maxhistory));
  cout << "Error";
  cout << endl;

  int k=0;
  for(unsigned int i=0; i < data.size(); i++){
    const Matrix& nomout = out(data,i);
    if(k<=maxhistory){ // copy the 10 timesteps (from start on) into pred
      preddata.push_back(nomout);
      cout << (nomout^T) << (nomout^T) << 0 << endl;
    }else{
      const Matrix& sensors = inp(preddata, k);
      const Matrix& result  = net.process(sensors); // activate with next data in order judge prediction
      double diff = (nomout - result).multTM().val(0,0);
      cout << (result^T) << (nomout^T) << diff << endl;
      preddata.push_back(result);
    }
    k++;
    if(k>iterations) {
      k=0;
      preddata.clear();
      i+=skip;
      for(int n=0; n<5; n++) cout << Matrix(1,outputdim*2) << 0 << endl;
    }
  }
  return 0;
}


