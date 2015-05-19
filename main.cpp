#include "QtKittiVisualizer.h"

#include <QApplication>

int main (int argc, char *argv[])
{
    QApplication a (argc, argv);
    KittiVisualizerQt w(NULL, argc, argv);
    w.show ();
    return a.exec ();
}
