// -*- C++ -*-
#ifdef GNUPLOT_ONLY_INCLUDES

#include <unistd.h>

#else

static FILE* OpenGnuplot(){
    return popen("gnuplot -geometry 400x300 -noraise >/dev/null 2>/dev/null","w");
}
static void CloseGnuplot(FILE* pipe){
    pclose(pipe);
}

#endif
