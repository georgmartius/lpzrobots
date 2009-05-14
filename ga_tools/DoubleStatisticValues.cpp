/*
 * DoubleStatisticValues.cpp
 *
 *  Created on: 14.05.2009
 *      Author: robot12
 */

#include "DoubleStatisticValues.h"

double avg(std::vector<double> values) {
	double avg=0;

	for(std::vector<double>::const_iterator iter = values.begin(); iter != values.end(); iter++) {
		avg += (*iter);
	}

	if(values.size()!=0)
	avg /= (double)values.size();

	return avg;
}

double max(std::vector<double> values) {
	double max=0;

	for(std::vector<double>::const_iterator iter = values.begin(); iter != values.end(); iter++) {
		if(max<(*iter))
			max=(*iter);
	}

	return max;
}

double min(std::vector<double> values) {
	double min=values[0];
	std::vector<double>::const_iterator iter = values.begin();

	for(iter++; iter != values.end(); iter++) {
		if(min>(*iter))
			min=(*iter);
	}

	return min;
}

double range(std::vector<double> values) {
	double dMax = max(values);
	double dMin = min(values);

	return dMax-dMin;
}

std::list<double>* sort(std::vector<double> values) {
	std::vector<double>::const_iterator iter;
	std::list<double>* list = new std::list<double>();

	for(iter = values.begin(); iter != values.end(); iter++) {
		list->push_back((*iter));
	}

	list->sort();

	return list;
}

double median(std::vector<double> values) {
	double median;
	std::list<double>* list = sort(values);
	int x;
	int num = values.size()/2;
	std::list<double>::const_iterator iter = list->begin();

	iter = list->begin();
	for(x=0;x<num;x++) iter++;

	if(values.size() % 2 == 0) {
		median = (*iter);
		iter--;
		median += (*iter);
		median *= 0.5;
	}

	else {
		median = (*iter);
	}

	return median;
}

double quartil1(std::vector<double> values) {
	double q;
	std::list<double>* list = sort(values);
	int x;
	int num = values.size()/4;
	std::list<double>::const_iterator iter = list->begin();

	iter = list->begin();
	for(x=0;x<num;x++) iter++;

	if(values.size() % 4 == 0) {
		q = (*iter);
		iter--;
		q += (*iter);
		q *= 0.5;
	}
	else {
		q = (*iter);
	}

	return q;
}

double quartil3(std::vector<double> values) {
	double q;
	std::list<double>* list = sort(values);
	int x;
	int num = values.size()/4*3;
	std::list<double>::const_iterator iter = list->begin();

	iter = list->begin();
	for(x=0;x<num;x++) iter++;

	if(values.size() % 4 == 0) {
			q = (*iter);
			iter--;
			q += (*iter);
			q *= 0.5;
		}
		else {
			q = (*iter);
		}

	return q;
}

double iqr(std::vector<double> values) {
	double q1 = quartil1(values);
	double q3 = quartil3(values);

	return q3-q1;
}

double whisker(double factor,std::vector<double> values) {
	double dIqr = iqr(values);

	return factor * dIqr;
}
