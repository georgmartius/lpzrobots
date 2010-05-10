
#include "stl_adds.h"
#include <stdio.h>

namespace std {
  
  string itos(int i){
    char str[10];
    sprintf(str,"%i", i);
    return string(str);
  }

}
