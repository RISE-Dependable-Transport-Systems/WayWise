#!/bin/sh
find . -type f -iregex ".*\.cpp" -exec cpp -I /usr/include/x86_64-linux-gnu/qt5/QtCore/ -I /usr/include/x86_64-linux-gnu/qt5/QtSerialPort/ -I /usr/include/x86_64-linux-gnu/qt5/QtNetwork/ -I /usr/include/x86_64-linux-gnu/qt5/QtGui/ -I /usr/include/x86_64-linux-gnu/qt5/QtPrintSupport/ -I /usr/include/x86_64-linux-gnu/qt5/QtWidgets/ -I /usr/include/x86_64-linux-gnu/qt5/ -fPIC {} -o test.o \;
rm test.o

