#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char*argv[])
{
	QApplication a(argc, argv);
	PCLViewer w; 	// PCLViewer is a class
	w.show();

	return a.exec();
}