#ifndef __STL_MAP_H
#define __STL_MAP_H

/* this file sets the HashSet and HashMap defines to make
 the old extension and the new tr1 for new and old gcc versions */

#if __GNUC__ < 4 
#include <ext/hash_set>
#include <ext/hash_map>
#define HashMap __gnu_cxx::hash_map
#define HashSet __gnu_cxx::hash_set

#else
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#define HashMap std::tr1::unordered_map
#define HashSet std::tr1::unordered_set
#endif


#endif
