#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QApplication>
#include <QCoreApplication>
#include <string>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void clearLayout(QLayout *layout);
    ~MainWindow();

private slots:
    void on_pushButton_addCheckbox_clicked();
    void on_pushButton_addCheckboxes_clicked();

private:
    Ui::MainWindow *ui;
    QWidget* m_checkboxes;
    QVBoxLayout* m_layout;
    int m_count;
    int m_checkBoxSize;
};

#endif // MAINWINDOW_H
