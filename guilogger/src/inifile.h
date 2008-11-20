/***************************************************************************
                          inifile.h  -  description
                             -------------------
    begin                : Tue Oct 3 2000
    copyright            : (C) 2000 by Georg Martius
    email                : georg.martius@mediennetzwerk.de
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef INIFILE_H
#define INIFILE_H

#include<qstring.h>
#include<qfile.h>
#include<q3ptrlist.h>

#define EMPTY 0
#define COMMENT 1
#define SECTION 2
#define VAR 3


class IniSection;
class IniVar;

class IniFile {
public: 
	IniFile();
	IniFile(QString _filename);
	~IniFile();

	bool Load();  //liest Datei ein und speichert alle Infos in den Variablen
	bool Save();  // Speichert alle Infos wieder ab. ACHTUNG eine vorhandene Datei wird ueberschrieben.
	void Clear(); // Loescht alle Infos aus den Vars. Aus Sicherheitsgründen wird der Dateiname intern auf "" gesetzt
	
  void setFilename(QString _filename);
  QString getFilename();

	
	void setComment(QString _comment);
	void addComment(QString _comment);
	QString getComment();

	// liefert eine Section mit dem Namen zurueck, beim direkt folgenden Aufruf gibt sie die nächste
	// Section mit den Eigenschaften.(bei _next)
	// return: false wenn keine Section mit dem Namen (mehr bei _next)
	bool getSection(IniSection& _section,QString _name,bool _next);
        IniSection *addSection(QString name);
        void delSection(IniSection* _section);

        // returns the value of the variable in the given section and default if it does not exist
        QString getValueDef(QString _section, QString _var, QString _default);
        
	Q3PtrList <IniSection> sections;	
	
private:
  // ermittelt Zeilentyp: siehe defines (COMMENT;SECTION;VAR)
  //  gibt gleich die gefundenen Strings zurueck (Sectionname oder varname,warvalue,varcomment
  int getLineType(QString _line,QString &str1,QString &str2, QString &str3);

  QString filename;
  QString comment;

  QFile file;

  bool bloaded;
};


class IniSection {
public:
	IniSection();
        IniSection(QString _name);
	~IniSection();
	
	void    setName(QString _name);
	QString getName();
	void    setComment(QString _comment);
	void    addComment(QString _addcomment);
	QString getComment();
	

	// liefert Var mit dem Namen
	// return: false wenn keine Var mit dem Namen
	bool getVar( IniVar& _var, QString _name);
        void addValue(QString name, QString value, QString comment = QString());
        void delVar(IniVar* _var);
        
  Q3PtrList <IniVar> vars;

  bool operator== (IniSection& _section); // Vergleicht nur Namen!
  void copy (IniSection& _section); // kopiert eigenen Inhalt in _section)

	
private:
  QString name;
  QString comment;

};

class IniVar {
public:
	IniVar();
	IniVar(QString _name,QString _value,QString _comment);
	~IniVar();
	
	void    setName(QString _name);
	QString getName();
	void    setValue(QString _value);
        QString getValue();
	void    setComment(QString _comment);
	QString getComment();
	
	 bool operator== (IniVar& _var); // Vergleicht nur Namen!
	 void copy (IniVar& _var);
	
	
private:
  QString name;
  QString value;
  QString comment;
};



#endif
