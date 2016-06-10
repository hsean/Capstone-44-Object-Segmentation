#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_hello_released();
    void on_pushButton_goodbye_released();
    void on_pushButton_cheese_released();

private:
    Ui::MainWindow *ui;
    int m_count;
};

#endif // MAINWINDOW_H
