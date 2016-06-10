#include "mainwindow.h"
#include "ui_mainwindow.h"

// Constructor
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->textBrowser->setReadOnly(true);

    // variable initialization
    m_count = 0;

    // Connect buttons to textBrowser
    //connect(ui->pushButton_hello, SIGNAL (released()), this, SLOT (on_pushButton_hello_released()));
    //connect(ui->pushButton_goodbye, SIGNAL (released()), this, SLOT (on_pushButton_goodbye_released()));
    //connect(ui->pushButton_cheese, SIGNAL (released()), this, SLOT (on_pushButton_cheese_released()));
}

// Destructor
MainWindow::~MainWindow()
{
    delete ui;
}

/* EVENT HANDLERS */
void MainWindow::on_pushButton_hello_released()
{
  QString text = QString::number(m_count) + " Hello button pressed";
  ui->textBrowser->append(text);
  ++m_count;
}

void MainWindow::on_pushButton_goodbye_released()
{
  QString text = QString::number(m_count) + " Goodbye button pressed";
  ui->textBrowser->append(text);
  ++m_count;
}

void MainWindow::on_pushButton_cheese_released()
{
  QString text = QString::number(m_count) + " Cheese button pressed";
  ui->textBrowser->append(text);
  ++m_count;
}
