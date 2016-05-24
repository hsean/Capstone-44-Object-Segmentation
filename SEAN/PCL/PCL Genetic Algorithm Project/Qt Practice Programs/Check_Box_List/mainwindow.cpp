#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->textBrowser->setReadOnly(true);

    // initialize variables
    m_count = 0;
    m_checkBoxSize = 0;

    // setup scroll area
    m_checkboxes = new QWidget(ui->scrollArea); // create widget
    m_layout = new QVBoxLayout(m_checkboxes);  // create vertical layout container

    /*
    QCheckBox* mycheckBox[100];
    for(int i=0;i<100; ++i)
    {
        mycheckBox[i] = new QCheckBox(checkboxes);
        mycheckBox[i]->setGeometry(QRect(30,30,331,18));
        mycheckBox[i]->setText(QApplication::translate("BarkasikForm","my0",0, 1));

        layout->addWidget(mycheckBox[i]);
    }
    ui->scrollArea->setWidget(checkboxes);
    */
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::clearLayout(QLayout *layout)
{
    if(layout)
    {
        while(layout->count() > 0)
        {
            QLayoutItem* item=layout->takeAt(0);
            delete item->widget();
            delete item;
        }
    }
}

void MainWindow::on_pushButton_addCheckbox_clicked()
{
    // Add checkbox to scroll area
    QCheckBox* myCheckBox = new QCheckBox(m_checkboxes);
    QString checkText = "check box " + QString::number(m_count);
    myCheckBox->setGeometry(QRect(30,30,331,18));
    myCheckBox->setText(checkText);
    m_layout->addWidget(myCheckBox);
    ui->scrollArea->setWidget(m_checkboxes);

    // Add text to text browser
    QString text = QString::number(m_count) + " button pressed";
    ui->textBrowser->append(text);

    ++m_count;
}

void MainWindow::on_pushButton_addCheckboxes_clicked()
{
    // clear checkboxes in scroll area
    clearLayout(m_layout);

    // add 10 new checkboxes to layout
    QCheckBox* mycheckBox[10];
    QString checkBoxText = "check box " + QString::number(m_count);
    for(int i=0;i<10; ++i)
    {
        mycheckBox[i] = new QCheckBox(m_checkboxes);
        mycheckBox[i]->setGeometry(QRect(30,30,331,18));
        mycheckBox[i]->setText(checkBoxText);

        m_layout->addWidget(mycheckBox[i]);
    }
    ui->scrollArea->update();
    ++m_count;

    // Add text to text browser
    ui->textBrowser->append("10 checkboxes added");
}
