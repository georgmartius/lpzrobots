/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Joerg Weider   <joergweide84 at aol dot com> (robot12)               *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Joern Hoffmann <jhoffmann at informatik dot uni-leipzig dot de       *
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

//includes
#include <vector>
#include <algorithm>

//ga_tools includes
#include "TournamentSelectStrategy.h"
#include "Generation.h"
#include "Individual.h"

TournamentSelectStrategy::TournamentSelectStrategy() {
        // nothing
}

TournamentSelectStrategy::TournamentSelectStrategy(RandGen* random) {
        m_random = random;
}

TournamentSelectStrategy::~TournamentSelectStrategy() {
        // nothing
}

void TournamentSelectStrategy::select(Generation* oldGeneration, Generation* newGeneration) {
        //int live = oldGeneration->getCurrentSize() - oldGeneration->getKillRate();
        //int size = newGeneration->getSize() - newGeneration->getCurrentSize();
        int num = oldGeneration->getCurrentSize();
        int kill = num - oldGeneration->getSize();
        Individual* ind1;                                                                        // the 2 individual which are tested
        Individual* ind2;
        int r1,r2;
        //std::map<int,Individual*> storage;
        //std::map<int,Individual*>::iterator iter;
        std::vector<Individual*> storage;                                        //list with all living individuals
        std::vector<Individual*>::iterator iter;                        //iterator for the list
        double f1,f2;

        for(int y=0;y<num;y++) {                                                        //take all individual in the list.
                storage.push_back(oldGeneration->getIndividual(y));
        }

        for(int x=0; x<kill; x++) {                                //select two individual for the test
                r1 = ((int)(m_random->rand()*1000000.0f)) % num;                //2 random indices for the individual list
                r2 = r1;
                while(r2==r1) {
                        r2 = ((int)(m_random->rand()*1000000.0f)) % num;
                }

                //ind1 = oldGeneration->getIndividual(r1);
                //ind2 = oldGeneration->getIndividual(r2);
                ind1 = storage[r1];                                                                        //become the 2 individual
                ind2 = storage[r2];

                f1 = ind1->getFitness();                                                        //the fitness values of this individuals
                f2 = ind2->getFitness();

                f1*=f1;                // abs                                                                        //in absolute
                f2*=f2;                // abs

                if(f1<f2) {                                                                                        //the test and than kill the worse
                        /*if(storage[r1]==0)
                                storage[r1]=ind1;
                        else
                                x--;*/
                        storage.erase(find(storage.begin(),storage.end(),storage[r2]));
                }
                else {
                        /*if(storage[r2]==0)
                                storage[r2]=ind2;
                        else
                                x--;*/
                        storage.erase(find(storage.begin(),storage.end(),storage[r1]));
                }

                num--;
        }

        /*char buffer[1024];
        sprintf(buffer,"fff_%i.txt",newGeneration->getGenerationNumber());
        FILE* fff = fopen(buffer,"w");*/
        for(iter=storage.begin();iter!=storage.end();iter++) {                        //transfer the living individual in the new generation
                newGeneration->addIndividual(*iter);
                //fprintf(fff,"%lf\n",(*iter)->getFitness());
        }
        //fclose(fff);
}
