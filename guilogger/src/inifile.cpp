/***************************************************************************
                          inifile.cpp  -  description
                             -------------------
    begin                : Tue Oct 3 2000
    copyright            : (C) 2000 by Georg Martius
    email                : georg.martius@web.de
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "inifile.h"
#include <iostream>
using namespace std;
#include <qregexp.h>


IniFile::IniFile(){
  sections.setAutoDelete(true);
  bloaded=false;
}

IniFile::IniFile(QString _filename){
  sections.setAutoDelete(true);
  bloaded=false;
  setFilename(_filename);
}


IniFile::~IniFile(){
}

void IniFile::setFilename(QString _filename){
  filename=_filename;
}

QString IniFile::getFilename(){
  return filename;
}


bool IniFile::Load(){
  sections.clear();
  char buffer[1024];
  QString line;
  IniSection* momsection;
  IniVar* momvar;
  bool beforeFirstSection=true;
  int LineType;
  QString str1,str2,str3;

  file.setName(filename);
  if (!file.exists()){
#ifdef DEBUG
    cerr << "\nFile doesn't exist: " << filename.latin1();
#else

#endif
    return false;
  }
  if (!file.open( QIODevice::ReadOnly)) {
#ifdef DEBUG
    cerr << "\nCannot read File: " << filename.latin1();
#else

#endif
    return false;
  }
  // Zeile für Zeile auslesen und zuordnen
  while( (file.readLine(buffer,1024))>0 ){
    line=QString(buffer);
    str1="";
    str2="";
    str3="";
    LineType=getLineType(line,str1,str2,str3);
    switch (LineType){
      case EMPTY:
      break;
      case COMMENT:
        if (beforeFirstSection){
          comment.append(str1);
        }else{
          momsection->addComment(str1);
        }
      break;
      case SECTION:
        beforeFirstSection=false;
        momsection=new IniSection(str1);
        sections.append(momsection);
      break;
      case VAR:
        if (beforeFirstSection){ // wird als Kommentar gewertet
          comment.append(str1);
        }else{                    // Normale Variable
          momvar=new IniVar(str1,str2,str3);
          momsection->vars.append(momvar);
        }
      break;
    }
  }

  file.close();

  bloaded=true;
  return true;
}


int IniFile::getLineType( QString _line, QString &str1, QString &str2, QString &str3){
  QRegExp regexp;
  int start,len;

  if (_line.isEmpty()|| _line=="\n") return EMPTY;

  if (_line.find(QRegExp("^[;#]"),0)!=-1){
    str1=_line;
    return COMMENT;
  }

  regexp.setPattern("^\\[.+\\]");
  if ( (start=regexp.indexIn(_line,0))>=0){
    len=regexp.matchedLength();
    str1=_line.mid(start+1,len-2);
    return SECTION;
  }

  regexp.setPattern(".+=.+");
  if ( (start=regexp.indexIn(_line,0))>=0 ){
    len=regexp.matchedLength();
    regexp.setPattern(".+=");
    start=regexp.indexIn(_line,0);
    len=regexp.matchedLength();
    str1=_line.left(len-1);
    int start2=len;

    
    regexp.setPattern(".+[;#]");
    start=regexp.indexIn( _line, start2);
    len=regexp.matchedLength();
    if (start>=0){ // is there a comment at the end of the line?
      str2=_line.mid(start2,len-1);
      str3=_line.mid(start2+len-1);
    }else{  // no comment
      str2=_line.mid(start2, _line.length()-start2-1);
    }
    return VAR;
  }

  // kann sich nur um eine alleinstehende Variable handeln
  str1=_line;
  
  return VAR;
}


bool IniFile::Save(){
  if (filename.isEmpty()) return false;
  file.setName(filename);
  if (! file.open(QIODevice::WriteOnly)){
#ifdef DEBUG
    cerr << "\nCannot write File: " << filename.latin1();
#else

#endif
    return false;
  }

  IniSection* section;
  IniVar* var;
  QString line;

  // Durchgehen und alles reinschreiben
  file.writeBlock(comment.latin1(),comment.length());
  for(section=sections.first();section!=0;section=sections.next()){
    line="\n[";
    line.append(section->getName());
    line.append("]\n");
    file.writeBlock(line.latin1(),line.length());
    line=section->getComment();
    if (!line.isEmpty()) {
      line += "\n";
      file.writeBlock(line.latin1(),line.length());
    }
    for(var=section->vars.first();var!=0;var=section->vars.next()){
      line=var->getName();
      if (! var->getValue().isEmpty()){
        line.append("=");
        line.append(var->getValue());
        line.append(var->getComment());
//      }else{
        line.append("\n");
      }
      file.writeBlock(line.latin1(),line.length());
    }
  }

  file.close();

  return true;
}

void IniFile::Clear(){
  sections.clear();
  filename="";
  bloaded=false;
  comment="";
}



bool IniFile::getSection(IniSection& _section,QString _name,bool _next){
  static QString lastname;
  IniSection* sec;
  if (_next==false || (_next==true && _name!=lastname) ) {
    lastname ="";
    sec=sections.first();

  }else{
    sec=sections.next();
  }

  lastname=_name;

  for(;sec!=0;sec=sections.next()){
      if (sec->getName()==_name){  // gefunden
        sec->copy(_section);
        return true;
      }
  }
  return false;
}


IniSection *IniFile::addSection(QString name)
{   IniSection* sec = new IniSection(name);
    sections.append(sec);
    return sec;
}


void IniFile::delSection(IniSection* _section)
{   sections.remove(_section);
    _section = NULL;
}


QString IniFile::getValueDef(QString _section, QString _var, QString _default){
  IniSection sec;
  if(getSection(sec,_section,false)){
    IniVar var;
    if(sec.getVar(var,_var)){
      return var.getValue();
    } else 
      return _default;
  } else 
    return _default;

}

void IniFile::setComment(QString _comment){
  comment=_comment;
}

void IniFile::addComment(QString _addcomment){
  comment.append(_addcomment);
}

QString IniFile::getComment(){
  return comment;
}


//////////////////////////////////////////////////////////////////////////////////////////(
//SECTION

IniSection::IniSection(){
  vars.setAutoDelete(true);
}

IniSection::IniSection(QString _name){
  vars.setAutoDelete(true);
  setName(_name);
}

IniSection::~IniSection(){
}

void IniSection::setName(QString _name){
  name=_name;
}

QString IniSection::getName(){
  return name;
}

void IniSection::setComment(QString _comment){
  comment=_comment;
}

void IniSection::addComment(QString _addcomment){
  comment.append(_addcomment);
}

QString IniSection::getComment(){
  return comment;
}

bool IniSection::operator== (IniSection& _section){
  return name==_section.getName();
}


void IniSection::copy (IniSection& _section){
  _section.setName(name);
  _section.setComment(comment);
  _section.vars=vars; // Operator von Qt ueberladen
  _section.vars.setAutoDelete(false);
}


bool IniSection::getVar( IniVar& _var, QString _name){
  IniVar* tempvar;
  for(tempvar=vars.first();tempvar;tempvar=vars.next()){
    if (tempvar->getName()==_name){
      tempvar->copy(_var);
      return true;
    }
  }
  return false;
}


void IniSection::delVar(IniVar* _var)
{    vars.remove(_var);  // automatic deletion
     _var = NULL;
}


void IniSection::addValue(QString name, QString value,QString comment)
{   IniVar* tmpvar = new IniVar(name, value, comment);
    vars.append(tmpvar);
}


/////////////////////////////////////////////////////////////////////////////////////////
// VAR
IniVar::IniVar(){
}

IniVar::IniVar( QString _name, QString _value, QString _comment){
  setName(_name);
  setValue(_value);
  setComment(_comment);
}

IniVar::~IniVar(){
}

void IniVar::setName(QString _name){
  name=_name;
}

QString IniVar::getName(){
  return name;
}

void IniVar::setValue(QString _value){
  value=_value;
}

QString IniVar::getValue(){
  return value;
}

void IniVar::setComment(QString _comment){
  comment=_comment;
}

QString IniVar::getComment(){
  return comment;
}

bool IniVar::operator== (IniVar& _var){
  return name==_var.getName();
}

void IniVar::copy (IniVar& _var){
  _var.setName(name);
  _var.setComment(comment);
  _var.setValue(value);
}
