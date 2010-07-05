#ifndef __STL_MAP_H
#define __STL_MAP_H

/* this file make a backward compatiblity with the std::hash_set/hash_map for gcc <4 */

#if __GNUC__ < 4 
#include <ext/hash_set>
#include <ext/hash_map>
#define tr1::unordered_map hash_map
#define tr1::unordered_set hash_set
#else
#include <tr1/unordered_map>
#include <tr1/unordered_set>

#endif


#endif
