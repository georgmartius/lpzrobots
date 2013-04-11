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

#include "TemplateValue.h"

/*template<class Typ> TemplateValue<Typ>::TemplateValue() : IValue() {
        // nothing
}

template<class Typ> TemplateValue<Typ>::TemplateValue(Typ value) : IValue(), m_value(value) {
}

template<class Typ> TemplateValue<Typ>::~TemplateValue() {
        // nothing
}

template<class Typ> IValue* TemplateValue<Typ>::operator*(const IValue& value)const {
        TemplateValue<Typ>* newValue;

        const TemplateValue<Typ> castValue = dynamic_cast<const TemplateValue<Typ> >(value);
        if(castValue==0)
                return 0;

        newValue = new TemplateValue<Typ>(castValue.getValue()*m_value);

        return newValue;
}*/

/*template<class Typ> IValue* TemplateValue::operator+(const IValue& value)const {
        TemplateValue<Typ>* newValue;

        const TemplateValue<Typ>& castValue = dynamic_cast<const TemplateValue<Typ>& >(value);
        if(castValue==0)
                return 0;

        newValue = new TemplateValue<Typ>(castValue.getValue()+m_value);
}

template<class Typ> IValue* TemplateValue::operator-(const IValue& value)const {
        TemplateValue<Typ>* newValue;

        const TemplateValue<Typ>& castValue = dynamic_cast<const TemplateValue<Typ>& >(value);
        if(castValue==0)
                return 0;

        newValue = new TemplateValue<Typ>(m_value-castValue.getValue());
}

template<class Typ> IValue* TemplateValue::abs(void)const {
        TemplateValue<Typ>* newValue;
        Typ wert1;

        wert = m_value;
        if(wert<0)
                wert*=-1.0;

        newValue = new TemplateValue<Typ>(wert);
}

template<class Typ> IValue* TemplateValue::operator/(const IValue& value)const {
        TemplateValue<Typ>* newValue;

        const TemplateValue<Typ>& castValue = dynamic_cast<const TemplateValue<Typ>& >(value);
        if(castValue==0)
                return 0;

        newValue = new TemplateValue<Typ>(m_value/castValue.getValue());

        return newValue;
}

template<class Typ> IValue* TemplateValue::operator/(double value)const {
        TemplateValue<Typ>* newValue;

        const TemplateValue<Typ>& castValue = dynamic_cast<const TemplateValue<Typ>& >(value);
        if(castValue==0)
                return 0;

        newValue = new TemplateValue<Typ>(m_value/value);

        return newValue;
}*/
