#include "ffnncontroller.h"

using namespace std;
using namespace matrix;

FFNNController::FFNNController(const std::string& networkfilename, int history, bool input_only_x, unsigned int init_wait)
  : AbstractController("FFNNController", "$Id: ffnncontroller.cpp,v 1.10 2011/05/30 13:52:54 martius Exp $") ,
    history(history), buffersize(history+10), input_only_x(input_only_x), s4avg(1), t(0), init_wait(init_wait) {
  net = new MultiLayerFFNN( 0.01, std::vector<Layer>());
  FILE* f = fopen(networkfilename.c_str(), "r");
  if(!f || !net->restore(f)){
    cerr << "could not load network from file: " << networkfilename << endl;
    perror("");
    exit(1);
  }
  addParameterDef("s4avg",&s4avg, 1, 1, buffersize-1, "input averaging time window");
  initialised=false;
}

FFNNController::FFNNController(MultiLayerFFNN* net, int history, bool input_only_x,
                               unsigned int init_wait)
  : AbstractController("FFNNController", "$Id: ffnncontroller.cpp,v 1.10 2011/05/30 13:52:54 martius Exp $") ,
    history(history), buffersize(history+10), input_only_x(input_only_x), s4avg(1),
    t(0), init_wait(init_wait), net(net) {
  addParameterDef("s4avg",&s4avg, 1, 1, buffersize-1, "input averaging time window");
  initialised=false;

}


FFNNController::~FFNNController() {
  if(net) delete net;
}

void FFNNController::init(int sensornumber, int motornumber, RandGen* randGen){
  number_motors  = motornumber;
  number_sensors = sensornumber;

  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
  }

  int idim = (input_only_x ? assembleNetworkInputX(x_buffer, y_buffer)
                           : assembleNetworkInputXY(x_buffer,y_buffer) ).getM();
  // sensornumber*(history+1) + (input_only_x ? 0 : motornumber*(history));
  int odim = assembleNetworkOutput(Matrix((signed)net->getOutputDim(),1)).getM();
  if(idim != (signed)net->getInputDim() || motornumber != odim){
    cerr << "input/output dimension of network does not fit! expect: "
         << idim << "," <<  motornumber
         << " but network has: " << net->getInputDim() << "," << odim << endl;
    exit(1);
  }

  t=0;
  initialised = true;
}

void FFNNController::step(const sensor* sensors, int number_sensors, motor* motors, int number_motors){
  stepNoLearning(sensors, number_sensors, motors, number_motors);
}

void FFNNController::stepNoLearning(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);

  Matrix x(number_sensors,1,x_);

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);

  // averaging over the last s4avg values of x_buffer
  //  x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(s4avg));

  if(t > init_wait){
    // calculate controller values based on smoothed input values
    const Matrix& inp = input_only_x ? assembleNetworkInputX(x_buffer, y_buffer)
      : assembleNetworkInputXY(x_buffer, y_buffer);
    const Matrix& out = net->process(inp);

    // calculate controller values based on smoothed input values
    const Matrix& y = assembleNetworkOutput(out);

    // put new output vector in ring buffer y_buffer
    putInBuffer(y_buffer, y);

    // convert y to motor*
    y.convertToBuffer(y_, number_motors);
  }else{
    memset(y_,0,number_motors*sizeof(double));
  }


  t++;
}


// put new value in ring buffer
void FFNNController::putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay){
  buffer[(t-delay)%buffersize] = vec;
}

/// calculate time-smoothed values
matrix::Matrix FFNNController::calculateSmoothValues(const matrix::Matrix* buffer,
                                                     int number_steps_for_averaging_) const {
  // number_steps_for_averaging_ must not be larger than buffersize
  assert ((int)number_steps_for_averaging_ <= buffersize);

  Matrix result(buffer[t % buffersize]);
  for (int k = 1; k < number_steps_for_averaging_; k++) {
    result += buffer[(t - k) % buffersize];
  }
  result *= 1/((double) (number_steps_for_averaging_)); // scalar multiplication
  return result;
};


matrix::Matrix FFNNController::assembleNetworkInputXY(matrix::Matrix* xbuffer, matrix::Matrix* ybuffer) const {
  int tp = t+buffersize;
  Matrix m(xbuffer[tp%buffersize]);
  for(int i=1; i<=history; i++){
    m=m.above(xbuffer[(tp-i)%buffersize].above(ybuffer[(tp-i)%buffersize]));
  }
  return m;
}

matrix::Matrix FFNNController::assembleNetworkInputX(matrix::Matrix* xbuffer, matrix::Matrix* ybuffer) const {
  int tp = t+buffersize;
  Matrix m(xbuffer[tp%buffersize]);
  for(int i=1; i<=history; i++){
    m=m.above(xbuffer[(tp-i)%buffersize]);
  }
  return m;
}

matrix::Matrix FFNNController::assembleNetworkOutput(const matrix::Matrix& output) const {
  return output;
}


bool FFNNController::store(FILE* f) const {
  return net->store(f);
}

bool FFNNController::restore(FILE* f){
  return net->restore(f);
}


void FFNNController::notifyOnChange(const paramkey& key){
  s4avg= max(1,s4avg);
}



