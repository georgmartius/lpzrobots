#ifndef __STL_ADDS_H
#define __STL_ADDS_H

#include<list>
#include<string>
#include<algorithm>
#include<vector>

#define FOREACH(colltype, coll, it) for( colltype::iterator it = (coll).begin(); it!= (coll).end(); it++)
#define FOREACHC(colltype, coll, it) for( colltype::const_iterator it = (coll).begin(); it!= (coll).end() ; it++ )

/// contains some additions to the standard template library
namespace std {

  /// absolute function for all types
  template<typename T>
  inline T abs(T v)
  { return ((v>0)?v:-v); }

  /// += operators for list (list concat)
  template <class T, class A>
    list<T,A>& operator += (list<T,A>& l1, const list<T,A>& l2) { 
    l1.insert(l1.end(), l2.begin(), l2.end()); 
    return l1;
  }

  /// += operators for list (append)
  template <class T, class A>
    list<T,A>& operator += (list<T,A>& l1, const T& v) { 
    l1.push_back(v); 
    return l1;
  }

  /// + operators for lists (list concat)
  template <class T, class A>
    list<T,A> operator + (const list<T,A>& l1, const list<T,A>& l2) { 
    list<T,A> rv(l1.begin(),l1.end());
    rv += l2;
    return rv;
  }

  /// integer to string with default formating
  string itos(int i);  
  /// integer to string with printf formating string
  string itos(int i, const char *);  

  template<typename Col, typename T> 
  bool removeElement(Col& col, const T& elem){
    // search for element
    typename Col::iterator i = find(col.begin(), col.end(), elem); 
    if(i != col.end()){
      col.erase(i);
      return true;
    }else{
      return false;
    }
  }


  template <class T>
  struct join : public unary_function<T, void>
  {
    join(const T& delimit) : delimit(delimit), count(0) {}
    void operator() (const T& s) { 
      if(count==0){
        joined=s;
      }else{       
        joined += delimit + s;        
      }
      ++count; 
    }
    T delimit;
    T joined;
    int count;
  };
  
}

#endif
