
//#include <QApplication>
#include <qapplication.h>
#include "mainwindow.h"

using namespace lpzrobots;

int main(int argc, char *argv[])
{
        Q_INIT_RESOURCE(application);

        QApplication app(argc, argv);

  QString appPath = QString(argv[0]);
  MainWindow *mainWin = new MainWindow(appPath.mid(0, appPath.lastIndexOf("/")+1));

        mainWin->show();
        return app.exec();
}

