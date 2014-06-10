
#include "SimplePipeReader.h"
#include <string>
#include <signal.h>


SimplePipeReader::SimplePipeReader()
{

//   signal(SIGPIPE,SIG_IGN);
  if(debug) std::cout << "new SimplePipeReader()" << std::endl;
  input_line = new QTextStream ( stdin, QIODevice::ReadOnly );
  currentChannelLine = "";
  currentDataLine = "";


//   if (dynamic_cast<QFile*>(input_line->device()))
//      QObject::connect(input_line->device() ,SIGNAL(), this, SLOT());
}

void SimplePipeReader::run()
{
  if(debug) std::cout << "SimplePipeReader: start()" << std::endl;
  bool closing=false;
  QByteArray charList;
//   int counter = 0;
  while ( !closing ) {
    usleep(1000);
    QString line = input_line->readLine ();
    //       std::cout << "SimplePipeReader: read " << line.size() << " chars" << std::endl;
    if ( line.isEmpty() ) continue;
    else if ( line.startsWith ( "#QUIT" ) ) {
      if(debug)  std::cout << "SimplePipeReader: have seen #QUIT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      closing = true;
      break;
    } else if ( line.startsWith ( "#RESET" ) ) {
      if(debug) std::cout << "SimplePipeReader: have seen #RESET **************************" << std::endl;


    } else if ( line.startsWith ( "#V" ) ) { // video recording -> capture frames
      QStringList pieces = line.split(" ");
      if(pieces.length()<3) { std::cout << "got" << line.toStdString()
                                        << ", but missing parameters expect \"#V idx directory\"" << std::endl;
      }else{
        long int cnt=pieces.at(1).toLong();
        emit captureFrame(cnt, pieces.at(2));
      }
    } else if ( line.startsWith ( "#IN" ) ) {  // Name
      emit sourceName(line.mid(4));
    }

    if (debug) std::cout << "currDatalin: " << (currentDataLine.section(' ', 0, 0)).toDouble();
    if (debug) std::cout << "oldline: " << (line.section(' ', 0, 0)).toDouble() << std::endl;

    if ( line.startsWith ( "#C" ) ) {
      //cut the first 2 chars (#C)
      line = line.mid ( 3 );
      currentChannelLine = line;
      //         std::cout << "SimplePipeReader currentChannelLine: [" << currentChannelLine.toStdString() << "]" << std::endl;
    } else if ( line.startsWith ( "#" ) ) {
      continue;
    } else if ( (currentChannelLine.size() > 2)
                && (line.section(' ', 0, 0) != currentDataLine.section(' ', 0, 0))) {
      currentDataLine = line;
      emit newData(); //wenn timestamp geändert (erstes element)
    }

    //      if ( line.startsWith ( "#D" ) ) {
    //          //cut the first 2 chars (#C)
    //          line = line.mid ( 3 );
    //          currentDescriptionLine = line;
    ////         printf("SimplePipeReader currentDescriptionLine: [%s]\r\n",currentDescriptionLine.toStdString().c_str());
    //      }
  }

  if(debug) std::cout << "SimplePipeReader SIGNAL(finished())" << std::endl;
  emit(finished());
}

bool SimplePipeReader::readyForData()
{
  if (debug) std::cout << "SimplePipeReader: waitForChannelData() ------START" << std::endl;
  if (currentChannelLine.size()<2) {
      wait(100);
    return false;
  }
  return true;

}

std::list<std::string> SimplePipeReader::getChannelLine()
{
//   std::cout << "SimplePipeReader: getChannelLine()" << std::endl;
  std::list<std::string> tmp_list;

  QStringList string_list = currentChannelLine.split ( ' ' );
  for ( int i = 0;i < string_list.size();i++ ) {

      std::string s = ( ( string_list.at ( i ) ).toAscii() ).data();//toStdString;
//    std::cout << "SimplePipeReader: getChannelLine[" << s << "]****************" << std::endl;
      tmp_list.push_back ( s );
    }
//   std::cout << "SimplePipeReader: getChannelLine()..." << tmp_list.size() << std::endl;
  return tmp_list;
}

std::list<std::string> SimplePipeReader::getDescriptionLine()
{
//   std::cout << "SimplePipeReader: getChannelLine()" << std::endl;
  std::list<std::string> tmp_list;

  QStringList string_list = currentDescriptionLine.split ( ' ' );
  for ( int i = 0;i < string_list.size();i++ ) {

    std::string s = ( ( string_list.at ( i ) ).toAscii() ).data();//toStdString;
//     std::cout << "SimplePipeReader: getChannelLine[" << s << "]****************" << std::endl;
    if (s.size() > 0)
      tmp_list.push_back ( s );
  }
//   std::cout << "SimplePipeReader: getDescriptionLine()..." << tmp_list.size() << std::endl;
  return tmp_list;

}


std::list< double > SimplePipeReader::getDataLine()
{
//   std::cout << "SimplePipeReader: getDataLine()" << std::endl;
  std::list<double> tmp_list;
  bool success;
  QString s;

  if(currentDataLine.startsWith("#")){
      return std::list<double>();
  }
  QStringList string_list = currentDataLine.split ( ' ' );
  for ( int i = 0;i < string_list.size();i++ ) {
    s = string_list.at ( i );
    if (s.size() == 0) continue;
    double d = s.toDouble ( &success );
    if ( success ) tmp_list.push_back ( d );
    else {
      printf("Cant do string TO double!!!!!\r\n");
      return std::list<double>();
    }
  }
  return tmp_list;
}

SimplePipeReader:: ~SimplePipeReader()
{
        std::cout << "SimplePipeReader: ByeBye()" << std::endl;
}
