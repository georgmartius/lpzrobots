
//#include <QApplication>
#include <qapplication.h>
#include "mainwindow.h"


int main(int argc, char *argv[])
{
	Q_INIT_RESOURCE(application);

	QApplication app(argc, argv);
	MainWindow mainWin;
	mainWin.applicationPath.clear();
	if(argc >= 1) mainWin.applicationPath.append(argv[0]);

	mainWin.show();
	return app.exec();
}

