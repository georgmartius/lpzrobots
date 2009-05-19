/*
 * TemplateValueAnalysation.h
 *
 *  Created on: 15.05.2009
 *      Author: robot12
 */

#ifndef TEMPLATEVALUEANALYSATION_H_
#define TEMPLATEVALUEANALYSATION_H_

#define ANALYSATION_CONTEXT TemplateValueAnalysation<type,zero,lower,higher,doubleDiv,doubleMul,add,sub,mul,div>

#include <vector>
#include <list>

template
<class type>
bool defaultLower(const type& a, const type& b) {
	if(a<b)
		return true;

	return false;
}

template
<class type>
type defaultDoubleDiv(const type& a, const double& b) {
	return a/b;
}

template
<class type>
type defaultDoubleMul(const type& a, const double& b) {
	return a*b;
}

template
<class type>
bool defaultHigher(const type& a,const type& b) {
	if(a>b)
		return true;

	return false;
}

template
<class type>
type defaultAdd(const type& a, const type& b) {
	return a+b;
}

template
<class type>
type defaultSub(const type& a, const type& b) {
	return a-b;
}

template
<class type>
type defaultMul(const type& a, const type& b) {
	return a*b;
}

template
<class type>
type defaultDiv(const type& a, const type& b) {
	return a/b;
}

double defaultZero() {
	return 0.0;
}

template
<class type,
type zero(void),
bool lower(const type&, const type&)=defaultLower<type>,
bool higher(const type&, const type&)=defaultHigher<type>,
type doubleDiv(const type&, const double&)=defaultDoubleDiv<type>,
type doubleMul(const type&, const double&)=defaultDoubleMul<type>,
type add(const type&, const type&)=defaultAdd<type>,
type sub(const type&, const type&)=defaultSub<type>,
type mul(const type&, const type&)=defaultMul<type>,
type div(const type&, const type&)=defaultDiv<type> >
class TemplateValueAnalysation {
public:
	TemplateValueAnalysation(std::vector<type>& values) : m_vector(values), m_listCreated(false), m_list() {}
	~TemplateValueAnalysation() {}

	type getAvg() {
		type avg=zero();
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter;

		for(iter = m_vector.begin(); iter != m_vector.end(); iter++) {
			avg = add(avg,(*iter));
		}

		if(m_vector.size()!=0)
			avg = doubleDiv(avg,m_vector.size());

		return avg;
	}

	type getMin() {
		type min = m_vector[0];
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter = m_vector.begin();

		for(iter++; iter != m_vector.end(); iter++) {
			if(lower((*iter),min))
				min=(*iter);
		}

		return min;
	}

	type getMax() {
		type max = m_vector[0];
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter = m_vector.begin();

		for(iter++; iter != m_vector.end(); iter++) {
			if(lower(max,(*iter)))
				max=(*iter);
		}

		return max;
	}

	type getRange() {
		type dMax = getMax();
		type dMin = getMin();

		return sub(dMax,dMin);
	}

	type getMedian() {
		type median;
		int x;
		int num = m_vector.size()/2;
		std::_List_iterator<TYPE_SAVE> iter;

		if(!m_listCreated)
			sort();

		iter = m_list.begin();
		for(x=0;x<num;x++) iter++;

		if(m_vector.size() % 2 == 0) {
			median = (*iter->pointer);
			iter--;
			median = add(median,(*iter->pointer));
			median = doubleMul(median,0.5);
		}

		else {
			median = (*iter->pointer);
		}

		return median;
	}

	type getQuartil1() {
		type q;
		int x;
		int num = m_vector.size()/4;
		std::_List_iterator<TYPE_SAVE> iter;

		if(!m_listCreated)
			sort();

		iter = m_list.begin();
		for(x=0;x<num;x++) iter++;

		if(m_vector.size() % 4 == 0) {
			q = (*iter->pointer);
			iter--;
			q = add(q,(*iter->pointer));
			q = doubleMul(q,0.5);
		}
		else {
			q = (*iter->pointer);
		}

		return q;
	}

	type getQuartil3() {
		type q;
		int x;
		int num = m_vector.size()*3/4;
		std::_List_iterator<TYPE_SAVE> iter;

		if(!m_listCreated)
			sort();

		iter = m_list.begin();
		for(x=0;x<num;x++) iter++;

		if(m_vector.size() % 4 == 0) {
			q = (*iter->pointer);
			iter--;
			q = add(q,(*iter->pointer));
			q = doubleMul(q,0.5);
		}
		else {
			q = (*iter->pointer);
		}

		return q;
	}

	type getIQR() {
		type q1 = getQuartil1();
		type q3 = getQuartil3();

		return sub(q3,q1);
	}

	type getWhisker(double factor) {
		type dIqr = getIQR();

		return doubleMul(dIqr,factor);
	}

	type getWhisker1(double factor) {
		type dW = getWhisker(factor);
		type dQ1 = getQuartil1();
		type dBorder = sub(dQ1,dW);
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dBorder) && iter!=m_list.end()) iter++;

		if(iter==m_list.end())		// ERROR
			return zero();

		return (*iter->pointer);
	}

	type getWhisker3(double factor) {
		type dW = getWhisker(factor);
		type dQ3 = getQuartil3();
		type dBorder = add(dQ3,dW);
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dBorder) && iter!=m_list.end()) iter++;

		iter--;

		return (*iter->pointer);
	}

	unsigned int getNumExtrems(double factor) {
		unsigned int result = 0;
		type dW1 = getWhisker1(factor);
		type dW3 = getWhisker3(factor);
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dW1) && iter!=m_list.end()) {
			iter++;
			result++;
		}

		while(!higher((*iter->pointer),dW3) && iter!=m_list.end()) {
			iter++;
		}

		while(iter!=m_list.end()) {
			iter++;
			result++;
		}

		return result;
	}

	type getExtrem(double factor, unsigned int i) {
		type dW1 = getWhisker1(factor);
		type dW3 = getWhisker3(factor);
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dW1) && iter!=m_list.end()) {
			i--;

			if(i==0)
				return (*iter->pointer);

			iter++;
		}

		while(!higher((*iter->pointer),dW3) && iter!=m_list.end()) {
			iter++;
		}

		while(iter!=m_list.end()) {
			i--;

			if(i==0)
				return (*iter->pointer);

			iter++;
		}

		return zero();		//not found
	}

protected:

	struct TYPE_SAVE {
		TYPE_SAVE(type& a) : pointer(&a) {}

		type* pointer;

		bool operator<(const TYPE_SAVE& other) {
			return lower((*pointer),(*other.pointer));
		}
	};

	std::vector<type>& m_vector;
	std::list<TYPE_SAVE> m_list;
	bool m_listCreated;

	void sort(void) {
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter;

		for(iter = m_vector.begin(); iter != m_vector.end(); iter++) {
			m_list.push_back(TYPE_SAVE((*iter)));
		}

		m_list.sort();
		m_listCreated = true;
	}
};

#endif /* TEMPLATEVALUEANALYSATION_H_ */
