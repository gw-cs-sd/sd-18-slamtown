#include <QApplication>
#include "include/prototyper_window.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  pcl::OpenNIGrabber grabber ("#1");
  Prototyper_Window proto_win(grabber);
  proto_win.show();
  return app.exec();
}
