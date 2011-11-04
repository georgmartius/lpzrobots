/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#ifndef TEMPLATEVALUEANALYSATION_H_
#define TEMPLATEVALUEANALYSATION_H_

//macros
#define ANALYSATION_CONTEXT TemplateValueAnalysation<type,zero,lower,higher,doubleDiv,doubleMul,add,sub,mul,div>
#define DOUBLE_ANALYSATION_CONTEXT TemplateValueAnalysation<double,defaultZero>

//includes
#include <vector>
#include <list>

/**
 * default function for "lower as" operation.
 * @param a operator a
 * @param b operator b
 * @return (bool) true if a lower than b
 */
template
<class type>
bool defaultLower(const type& a, const type& b) {
	if(a<b)
		return true;

	return false;
}

/**
 * default function for divide by a double value.
 * @param a operator a
 * @param b (double) the double value
 * @return the result of the division
 */
template
<class type>
type defaultDoubleDiv(const type& a, const double& b) {
	return a/b;
}

/**
 * default function for mul. with a double value.
 * @param a operator a
 * @param b (double) the double value
 * @return the result of the mul.
 */
template
<class type>
type defaultDoubleMul(const type& a, const double& b) {
	return a*b;
}

/**
 * default function for "higher than" operation.
 * @param a operator a
 * @param b operator b
 * @return (bool) true if a higher than b
 */
template
<class type>
bool defaultHigher(const type& a,const type& b) {
	if(a>b)
		return true;

	return false;
}

/**
 * default function for add two values
 * @param a operator a
 * @param b operator b
 * @return the result of the add.
 */
template
<class type>
type defaultAdd(const type& a, const type& b) {
	return a+b;
}

/**
 * default function for sub. two values
 * @param a operator a
 * @param b operator b
 * @return the result of the sub.
 */
template
<class type>
type defaultSub(const type& a, const type& b) {
	return a-b;
}

/**
 * default function for mul. of two values
 * @param a operator a
 * @param b operator b
 * @return the result of the mul.
 */
template
<class type>
type defaultMul(const type& a, const type& b) {
	return a*b;
}

/**
 * default function for div. of two values
 * @param a operator a
 * @param b operator b
 * @return the result of the div.
 */
template
<class type>
type defaultDiv(const type& a, const type& b) {
	return a/b;
}

/**
 * default function for zero double value
 * @return (double) 0.0
 */
double defaultZero();

/**
 * This template class give you some methods to calculate some statistical values like average, min, max, upper quartil,
 * lower quartil and many more.
 *
 * All functions are implemented in the header because by generating the library it isn't known which type are later used.
 * And a later compiling needs the implementation two!
 */
template
<class type,															//data type of the calculation
type zero(void),														//gives the zero of this data type back
bool lower(const type&, const type&)=defaultLower<type>,				//test if one element of the type lower than the other
bool higher(const type&, const type&)=defaultHigher<type>,			//test if one element of the type higher than the other
type doubleDiv(const type&, const double&)=defaultDoubleDiv<type>,	//divide a element of the type by a double value
type doubleMul(const type&, const double&)=defaultDoubleMul<type>,	//mul. a element of the type by a double value
type add(const type&, const type&)=defaultAdd<type>,					//add two elements of type
type sub(const type&, const type&)=defaultSub<type>,					//sub two elements of type
type mul(const type&, const type&)=defaultMul<type>,					//mul two elements of type
type div(const type&, const type&)=defaultDiv<type> >				//div two elements of type
class TemplateValueAnalysation {
public:
	/**
	 * constructor
	 * Needs a set of values for which the statistical values will calculate.
	 * @param values (vector<type>& the set)
	 */
	TemplateValueAnalysation(std::vector<type>& values) : m_vector(values), m_list(), m_listCreated(false) {}

	/**
	 * default destructor
	 */
	~TemplateValueAnalysation() {}

	/**
	 * this function calculate the average of the giving set
	 *
	 * It used zero, add and doubleDiv to calculate the average.
	 * @return the average
	 */
	type getAvg() {
		type avg=zero();																		//by begin the average is zero
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter;

		for(iter = m_vector.begin(); iter != m_vector.end(); iter++) {
			avg = add(avg,(*iter));																//for all elements in the set add it to the average.
																								//So we become the sum. of all elements in the set.
		}

		if(m_vector.size()!=0)
			avg = doubleDiv(avg,m_vector.size());												//now devide the sum by the count of elements in the set.

		return avg;																				//return the result
	}

	/**
	 * this function search the min. value in the set
	 *
	 * For this it use lower
	 * @return the minimum
	 */
	type getMin() {
		type min = m_vector[0];			//the lowest element is at begin the first element
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter = m_vector.begin();

		for(iter++; iter != m_vector.end(); iter++) {
			if(lower((*iter),min))		//if a element lower than min, so reset the min to the lower value.
				min=(*iter);
		}

		return min;						//return the lowest element
	}

	/**
	 * this function search the max. value in the set
	 *
	 * For this it use lower
	 * @return the minimum
	 */
	type getMax() {
		type max = m_vector[0];			//the highest element is at begin the first element
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter = m_vector.begin();

		for(iter++; iter != m_vector.end(); iter++) {
			if(lower(max,(*iter)))		//if a element higher than max, so reset the max to the higher value.
				max=(*iter);
		}

		return max;						//return the lowest element
	}

	/**
	 * this function calculate the range of all values. For this he search the mibn and max and calculate from this values.
	 *
	 * use sub
	 * @return the range of the values in the set
	 */
	type getRange() {
		type dMax = getMax();			//become max
		type dMin = getMin();			//become min

		return sub(dMax,dMin);			//range=max-min
	}

	/**
	 * this function search the median of the giving set of values.
	 *
	 * use add and doubleMul
	 * @return the median
	 */
	type getMedian() {
		type median;
		int x;
		int num = m_vector.size()/2;
		std::_List_iterator<TYPE_SAVE> iter;

		if(!m_listCreated)
			sort();								//sort the set. to define the middle

		iter = m_list.begin();					//go to the middle
		for(x=0;x<num;x++) iter++;

		if(m_vector.size() % 2 == 0) {			//if the real middle between two values add this values and calculate the arithmetical middle.
			median = (*iter->pointer);
			iter--;
			median = add(median,(*iter->pointer));
			median = doubleMul(median,0.5);
		}

		else {
			median = (*iter->pointer);			//else gives the middle back
		}

		return median;
	}

	/**
	 * this function calculate the under quartil
	 *
	 * use add and doubleMul
	 * @return the under quartil
	 */
	type getQuartil1() {
		type q;
		int x;
		int num = m_vector.size()/4;
		std::_List_iterator<TYPE_SAVE> iter;

		if(!m_listCreated)
			sort();								//sort the set.

		iter = m_list.begin();					//go to the under quartil
		for(x=0;x<num;x++) iter++;

		if(m_vector.size() % 4 == 0) {			//if the real under quartil between two values add this values and calculate the arithmetical middle.
			q = (*iter->pointer);
			iter--;
			q = add(q,(*iter->pointer));
			q = doubleMul(q,0.5);
		}
		else {									//else return the quartil
			q = (*iter->pointer);
		}

		return q;
	}

	/**
	 * this function calculate the upper quartil.
	 *
	 * use add and doubleMul
	 * @return the upper quartil
	 */
	type getQuartil3() {
		type q;
		int x;
		int num = m_vector.size()*3/4;
		std::_List_iterator<TYPE_SAVE> iter;

		if(!m_listCreated)
			sort();								//sort the set.

		iter = m_list.begin();
		for(x=0;x<num;x++) iter++;				//go to the upper quartil

		if(m_vector.size() % 4 == 0) {			//if the real upper quartil between two values add this values and calculate the arithmetical middle.
			q = (*iter->pointer);
			iter--;
			q = add(q,(*iter->pointer));
			q = doubleMul(q,0.5);
		}
		else {									//else return the quartil
			q = (*iter->pointer);
		}

		return q;
	}

	/**
	 * this function calculate the range between the upper and under quartil. This range is called inter-quartil-range.
	 *
	 * use sub
	 * @return the IQR
	 */
	type getIQR() {
		type q1 = getQuartil1();			//calculate the under quartil
		type q3 = getQuartil3();			//calculate the upper quartil

		return sub(q3,q1);					//IQR = Q3 - Q1
	}

	/**
	 * this function calculate the whisker distance
	 *
	 * use doubleMul
	 * @param factor (double) the factor for this distance
	 * @return the result
	 */
	type getWhisker(double factor) {
		type dIqr = getIQR();			//calculate the IQR

		return doubleMul(dIqr,factor);	//WHISKER distance is factor*IQR
	}

	/**
	 * this function search the lowest value in the set which is from under quartil inside the whisker distance
	 *
	 * use sub, lower and zero
	 * @param factor (double) this factor is for calculating the whisker distance.
	 * @return the lowest value inside
	 */
	type getWhisker1(double factor) {
		type dW = getWhisker(factor);				//calculate the whisker distance
		type dQ1 = getQuartil1();					//calculate the under quartil
													//TODO for optimization: getWhisker use getIQR and this calculate Q1 so it will be calculate two times!!!
		type dBorder = sub(dQ1,dW);					//where is the border for the lowest value
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dBorder) && iter!=m_list.end()) {		//search
			iter++;
			if(iter==m_list.end())
				break;
		}

		if(iter==m_list.end())		// ERROR
			return zero();

		return (*iter->pointer);
	}

	/**
	 * this function search the highest value in the set which is from the upper quartil inside the whisker distance
	 *
	 * use add and lower
	 * @param factor (double) this factor is for calculating the whisker distance.
	 * @return the highest value inside
	 */
	type getWhisker3(double factor) {
		type dW = getWhisker(factor);				//calculate the whisker distance
		type dQ3 = getQuartil3();					//calculate the upper quartil
													//TODO for optimization: getWhisker use getIQR and this calculate Q3 so it will be calculate two times!!!
		type dBorder = add(dQ3,dW);					//where is the border for the lowest value
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dBorder) && iter!=m_list.end()) {		//search
			iter++;
			if(iter==m_list.end())
				break;
		}

		if(iter!=m_list.begin())
			iter--;

		return (*iter->pointer);
	}

	/**
	 * this function give you the number of elements in the giving set, which aren't in the whisker distance
	 *
	 * use lower and higher
	 * @param factor (double) this factor is for calculating the whisker distance
	 * @return (int) the number of extreme values in the set
	 */
	unsigned int getNumExtrems(double factor) {
		unsigned int result = 0;
		type dW1 = getWhisker1(factor);								//find W1
		type dW3 = getWhisker3(factor);								//find W3
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dW1) && iter!=m_list.end()) {	//count all elements which are lower than W1
			iter++;
			if(iter==m_list.end())
				break;
			result++;
		}

		while(!higher((*iter->pointer),dW3) && iter!=m_list.end()) {	//go from W1 to W3
			iter++;
			if(iter==m_list.end())
				break;
		}

		while(iter!=m_list.end()) {									//count all element which are higher than W3
			iter++;
			result++;
		}

		return result;
	}

	/**
	 * this function gives one extreme value back
	 *
	 * use lower, higher and zero
	 * @param factor (double) the factor for the whisker distance
	 * @param i (unsigned int) the index of the searched extreme value
	 * @return the founded extreme value
	 */
	type getExtrem(double factor, unsigned int i) {
		type dW1 = getWhisker1(factor);							//find W1
		type dW3 = getWhisker3(factor);							//find W3
		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),dW1) && iter!=m_list.end()) {			//search in lower area
			i--;

			if(i==0)
				return (*iter->pointer);

			iter++;
			if(iter==m_list.end())
				break;
		}

		while(!higher((*iter->pointer),dW3) && iter!=m_list.end()) {		//go from W1 to W3
			iter++;
			if(iter==m_list.end())
				break;
		}

		while(iter!=m_list.end()) {											//search in upper area
			i--;

			if(i==0)
				return (*iter->pointer);

			iter++;
		}

		return zero();		//not found										//if not found return zero
	}

	/**
	 * this function search the value which is next to zero.
	 *
	 * use zero, sub and lower
	 * @return the best value
	 */
	type getBest(void) {
		type z = zero();											//test value is zero
		type* l;
		type* h;

		if(!m_listCreated)
			sort();													//sort the list

		std::_List_iterator<TYPE_SAVE> iter = m_list.begin();

		while(lower((*iter->pointer),z) && iter!=m_list.end()) {	//search the element which is as first higher than zero
			iter++;
			if(iter==m_list.end())
				break;
		}

		if(iter!=m_list.end())
			//return (*iter->pointer);
			h = iter->pointer;

		else {
			iter--;
			//return (*iter->pointer);
			h = iter->pointer;
		}

		while(!lower((*iter->pointer),z) && iter!= m_list.begin()) {	//search the element which is as first lower than zero
			iter--;
			if(iter==m_list.begin())
				break;
		}

		l = iter->pointer;

		if(lower(sub(*h,z),sub(z,*l)))		//test which is next to zero
			return *h;
		else
			return *l;
	}

protected:

	/**
	 * help structur for sorting the set.
	 * define the lower than operator
	 */
	struct TYPE_SAVE {
		TYPE_SAVE(type& a) : pointer(&a) {}

		type* pointer;

		bool operator<(const TYPE_SAVE& other) {
			return lower((*pointer),(*other.pointer));
		}
	};

	/**
	 * this vector save the giving set.
	 */
	std::vector<type>& m_vector;

	/**
	 * this list saves the sorted set
	 */
	std::list<TYPE_SAVE> m_list;

	/**
	 * this variable remember if the sorted list is created
	 */
	bool m_listCreated;

	/**
	 * this function create the sorted list
	 */
	void sort(void) {
		__gnu_cxx::__normal_iterator<type*,std::vector<type,std::allocator<type> > > iter;

		for(iter = m_vector.begin(); iter != m_vector.end(); iter++) {			//fill the sorted list with elements of the help structur
			m_list.push_back(TYPE_SAVE((*iter)));
		}

		m_list.sort();				//sort the list
		m_listCreated = true;		//remember that it is created.
	}
};

#endif /* TEMPLATEVALUEANALYSATION_H_ */
