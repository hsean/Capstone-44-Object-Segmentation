#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
#include <librealsense/rs.hpp>


int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCLViewer w;
  w.show ();
  rs::log_to_console(rs::log_severity::warn);

  return a.exec ();
}
