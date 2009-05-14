/*
 * DoubleStatisticValues.h
 *
 *  Created on: 14.05.2009
 *      Author: robot12
 */

#ifndef DOUBLESTATISTICVALUES_H_
#define DOUBLESTATISTICVALUES_H_

#include <math.h>
#include <vector>
#include <list>

double avg(std::vector<double> values);
double max(std::vector<double> values);
double min(std::vector<double> values);
double range(std::vector<double> values);

std::list<double>* sort(std::vector<double> values);

double median(std::vector<double> values);
double quartil1(std::vector<double> values);
double quartil3(std::vector<double> values);
double iqr(std::vector<double> values);
double whisker(double factor,std::vector<double> values);

#endif /* DOUBLESTATISTICVALUES_H_ */
