
#include "SimplePipeReader.h"
#include <string>
#include <signal.h>


SimplePipeReader::SimplePipeReader()
{

//   signal(SIGPIPE,SIG_IGN);
  std::cout << "new SimplePipeReader()" << std::endl;
  input_line = new QTextStream ( stdin, QIODevice::ReadOnly );
  currentChannelLine = "";
  currentDataLine = "";


//   if (dynamic_cast<QFile*>(input_line->device()))
//      QObject::connect(input_line->device() ,SIGNAL(), this, SLOT());
}

void SimplePipeReader::run()
{
  std::cout << "SimplePipeReader: start()" << std::endl;
  bool closing=false;
  QByteArray charList;
//   int counter = 0;
  while ( !closing ) {
      QString line = input_line->readLine ( 500 );
//       std::cout << "SimplePipeReader: read " << line.size() << " chars" << std::endl;
      if ( line.isEmpty() ) continue;

    if ( line.startsWith ( "#QUIT" ) ) {
      std::cout << "SimplePipeReader: have seen #QUIT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      closing = true;
      break;
    }

    if ( line.startsWith ( "#RESET" ) ) {
      std::cout << "SimplePipeReader: have seen #RESET **************************" << std::endl;


    }

    if ( (currentChannelLine.size() > 2) && (currentDescriptionLine.size() > 2) ) {
          currentDataLine = line;
//       std::cout << "SimplePipeReader currentDataLine: [" << currentDataLine.toStdString() << "]" << std::endl;
          emit newData();
        }

      if ( line.startsWith ( "#C" ) ) {
          //cut the first 2 chars (#C)
          line = line.mid ( 3 );
          currentChannelLine = line;
//         std::cout << "SimplePipeReader currentChannelLine: [" << currentChannelLine.toStdString() << "]" << std::endl;
        }
      if ( line.startsWith ( "#D" ) ) {
          //cut the first 2 chars (#C)
          line = line.mid ( 3 );
          currentDescriptionLine = line;
//         printf("SimplePipeReader currentDescriptionLine: [%s]\r\n",currentDescriptionLine.toStdString().c_str());
      }
    }

  std::cout << "SimplePipeReader SIGNAL(finished())" << std::endl;
  emit(finished());
}

bool SimplePipeReader::readyForData()
{
   std::cout << "SimplePipeReader: waitForChannelData() ------START" << std::endl;
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

  QStringList string_list = currentDataLine.split ( ' ' );
  for ( int i = 0;i < string_list.size();i++ ) {
    s = string_list.at ( i );
    if (s.size() == 0) continue;
    double d = s.toDouble ( &success );
    if ( success ) tmp_list.push_back ( d );
    else {
      printf("Cant do string TO double!!!!!\r\n");
      tmp_list.push_back ( 2. );
    }
  }
  return tmp_list;
}

SimplePipeReader:: ~SimplePipeReader()
{
        std::cout << "SimplePipeReader: ByeBye()" << std::endl;
}

