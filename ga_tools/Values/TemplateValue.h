/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    joergweide84@aol.com (robot12)                                       *
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
 ***************************************************************************
 *                                                                         *
 *   This class is a implementation for the IValue interface. It is for    *
 *   standard data types concepted as a template class.                    *
 *                                                                         *
 *   $Log$
 *   Revision 1.9  2009-10-23 10:48:02  robot12
 *   bugfix in store and restore
 *
 *   Revision 1.8  2009/10/21 14:08:19  robot12
 *   add restore and store functions to the ga package
 *
 *   Revision 1.7  2009/07/21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.6  2009/06/26 13:08:25  robot12
 *   finishing Values and add some comments
 *
 *   Revision 1.5  2009/05/14 15:29:54  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.4  2009/05/12 13:29:24  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.3  2009/05/11 14:08:51  robot12
 *   patch some bugfix....
 *
 *   Revision 1.2  2009/05/06 13:28:22  robot12
 *   some implements... Finish
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/05/04 09:20:52  robot12
 *   some implements.. Finish --> first compile
 *
 *   Revision 1.1  2009/04/29 14:32:29  robot12
 *   some implements... Part4
 *
 *
 *
 ***************************************************************************/

#ifndef TEMPLATEVALUE_H_
#define TEMPLATEVALUE_H_

//includes
#include <string>

//ga_tools includes
#include "IValue.h"
#include "restore.h"

/**
 * general function to converrt a double value to a string
 * @param value (double) the value wath should convert
 * @return
 */
inline std::string doubleToString(double value) {
	char buffer[128];
	sprintf(buffer,"% .12lf",value);
	return buffer;
}

/**
 * template class for a IValue standard data type
 * needs the data type and a methode for string converting as template argument
 *
 * All parts are declared and defined in the header because the needed template implementations
 * are needed to the compiling time of the lib. To create new classes in an using program it
 * must be in the header.
 */
template<class Typ, std::string toString(Typ)=doubleToString>
class TemplateValue : public IValue {
public:
	/**
	 * constructor
	 * needs the value and a name (for IValue -> is default implemented as "templateValue")
	 * @param value (Typ) the value of this IValue
	 * @param name (string) the name
	 */
	TemplateValue(Typ value, std::string name = "templateValue") : IValue(name), m_value(value)  {}

	/**
	 * default destructor
	 */
	virtual ~TemplateValue()  {}

	/**
	 * this function can be used to read the standard data type.
	 * @return (Typ) the value
	 */
	inline Typ getValue(void)const {return m_value;}

	/**
	 * this function is to change the value.
	 * @param value
	 */
	inline void setValue(Typ value) {m_value=value;}

	/**
	 * the implementation of the mul operator, what is part of the interface.
	 * This function only accept TemplateValues or the same type like "Typ"
	 * @param value (const IValue&) the other part of the operation
	 * @return (IValue*) the result
	 */
	virtual IValue* operator*(const IValue& value)const {
		TemplateValue<Typ,toString>* newValue;

		//cast the IValue to TemplateValue of the same type like "Typ"
		const TemplateValue<Typ,toString>* castValue = dynamic_cast<const TemplateValue<Typ,toString>* >(&value);
		if(castValue==0)
			return 0;

		//multiplicate the values
		const Typ typeValue = castValue->getValue();
		newValue = new TemplateValue<Typ,toString>(m_value*typeValue);

		//return result
		return newValue;
	}

	/**
	 * the implementation of the add operator what is part of the interface.
	 * This function only accept TemplateValues or the same type like "Typ"
	 * @param value (const IValue&) the other part of the operation
	 * @return (IValue*) the result
	 */
	virtual IValue* operator+(const IValue& value)const {
		TemplateValue<Typ,toString>* newValue;

		//cast the IValue to TemplateValue of the same type like "Typ"
		const TemplateValue<Typ,toString>* castValue = dynamic_cast<const TemplateValue<Typ,toString>* >(&value);
		if(castValue==0)
			return 0;

		//add the values
		const Typ typeValue = castValue->getValue();
		newValue = new TemplateValue<Typ,toString>(m_value+typeValue);

		//return the result
		return newValue;
	}

	/**
	 * cast operatot to string
	 * use the convert methode.
	 * @return (string) the cast result
	 */
	virtual operator std::string(void)const {
		return toString(m_value);
	}

	/**
   * store the value in a file
   * @param f (FILE*) the file to store
   * @return (bool) true if all ok.
   */
  virtual bool store(FILE* f) const {
    RESTORE_GA_TEMPLATE<Typ> temp;

    //test
    if(f==NULL) {
      printf("\n\n\t>>> [ERROR] <<<\nNo File to store GA [temp value].\n\t>>> [END] <<<\n\n\n");
      return false;
    }

    temp.value = m_value;

    fprintf(f,"%i\n%s",(int)m_name.length(),m_name.c_str());

    for(unsigned int x=0;x<sizeof(RESTORE_GA_TEMPLATE<Typ>);x++) {
      fprintf(f,"%c",temp.buffer[x]);
    }

    return true;
  }

  /**
   * restore the value from a file
   * @param f (FILE*) the file where the value inside
   * @return (bool) true if all ok.
   */
  virtual bool restore(FILE* f) {
    RESTORE_GA_TEMPLATE<Typ> temp;
    char* buffer;
    int toread;

    //test
    if(f==NULL) {
      printf("\n\n\t>>> [ERROR] <<<\nNo File to restore GA [temp value].\n\t>>> [END] <<<\n\n\n");
      return false;
    }

    fscanf(f,"%i\n",&toread);
    buffer=new char[toread];
    for(int y=0;y<toread;y++){
      fscanf(f,"%c",&buffer[y]);
    }
    buffer[toread]='\0';
    m_name=buffer;
    delete[] buffer;

    for(unsigned int x=0;x<sizeof(RESTORE_GA_TEMPLATE<Typ>);x++) {
      fscanf(f,"%c",&temp.buffer[x]);
    }

    m_value = temp.value;

    return true;
  }

protected:
	/**
	 * the real value
	 */
	Typ m_value;

private:
	/**
	 * disable the default constructor
	 * @return
	 */
	TemplateValue() : IValue() {}
};

#endif /* TEMPLATEVALUE_H_ */
