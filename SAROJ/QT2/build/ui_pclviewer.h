/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_Axis;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *radioButton_x;
    QRadioButton *radioButton_y;
    QRadioButton *radioButton_z;
    QSpacerItem *verticalSpacer;
    QGroupBox *groupBox_ColorMode;
    QVBoxLayout *verticalLayout;
    QRadioButton *radioButton_BlueRed;
    QRadioButton *radioButton_GreenMagenta;
    QRadioButton *radioButton_WhiteRed;
    QRadioButton *radioButton_GreyRed;
    QRadioButton *radioButton_Rainbow;
    QSpacerItem *verticalSpacer_3;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_load;
    QPushButton *pushButton_save;
    QVTKWidget *qvtkWidget;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QString::fromUtf8("PCLViewer"));
        PCLViewer->setWindowModality(Qt::NonModal);
        PCLViewer->resize(896, 498);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout_3 = new QHBoxLayout(centralwidget);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox_Axis = new QGroupBox(centralwidget);
        groupBox_Axis->setObjectName(QString::fromUtf8("groupBox_Axis"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_Axis->sizePolicy().hasHeightForWidth());
        groupBox_Axis->setSizePolicy(sizePolicy);
        groupBox_Axis->setMinimumSize(QSize(180, 60));
        QFont font;
        font.setPointSize(16);
        font.setBold(true);
        font.setWeight(75);
        groupBox_Axis->setFont(font);
        horizontalLayout_2 = new QHBoxLayout(groupBox_Axis);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        radioButton_x = new QRadioButton(groupBox_Axis);
        radioButton_x->setObjectName(QString::fromUtf8("radioButton_x"));
        QFont font1;
        font1.setPointSize(16);
        font1.setBold(false);
        font1.setWeight(50);
        radioButton_x->setFont(font1);
        radioButton_x->setChecked(false);

        horizontalLayout_2->addWidget(radioButton_x);

        radioButton_y = new QRadioButton(groupBox_Axis);
        radioButton_y->setObjectName(QString::fromUtf8("radioButton_y"));
        radioButton_y->setFont(font1);
        radioButton_y->setChecked(true);

        horizontalLayout_2->addWidget(radioButton_y);

        radioButton_z = new QRadioButton(groupBox_Axis);
        radioButton_z->setObjectName(QString::fromUtf8("radioButton_z"));
        radioButton_z->setFont(font1);
        radioButton_z->setChecked(false);

        horizontalLayout_2->addWidget(radioButton_z);


        verticalLayout_2->addWidget(groupBox_Axis);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        groupBox_ColorMode = new QGroupBox(centralwidget);
        groupBox_ColorMode->setObjectName(QString::fromUtf8("groupBox_ColorMode"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(groupBox_ColorMode->sizePolicy().hasHeightForWidth());
        groupBox_ColorMode->setSizePolicy(sizePolicy1);
        groupBox_ColorMode->setMinimumSize(QSize(230, 180));
        groupBox_ColorMode->setFont(font);
        verticalLayout = new QVBoxLayout(groupBox_ColorMode);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        radioButton_BlueRed = new QRadioButton(groupBox_ColorMode);
        radioButton_BlueRed->setObjectName(QString::fromUtf8("radioButton_BlueRed"));
        radioButton_BlueRed->setFont(font1);

        verticalLayout->addWidget(radioButton_BlueRed);

        radioButton_GreenMagenta = new QRadioButton(groupBox_ColorMode);
        radioButton_GreenMagenta->setObjectName(QString::fromUtf8("radioButton_GreenMagenta"));
        radioButton_GreenMagenta->setFont(font1);

        verticalLayout->addWidget(radioButton_GreenMagenta);

        radioButton_WhiteRed = new QRadioButton(groupBox_ColorMode);
        radioButton_WhiteRed->setObjectName(QString::fromUtf8("radioButton_WhiteRed"));
        radioButton_WhiteRed->setFont(font1);

        verticalLayout->addWidget(radioButton_WhiteRed);

        radioButton_GreyRed = new QRadioButton(groupBox_ColorMode);
        radioButton_GreyRed->setObjectName(QString::fromUtf8("radioButton_GreyRed"));
        radioButton_GreyRed->setFont(font1);

        verticalLayout->addWidget(radioButton_GreyRed);

        radioButton_Rainbow = new QRadioButton(groupBox_ColorMode);
        radioButton_Rainbow->setObjectName(QString::fromUtf8("radioButton_Rainbow"));
        radioButton_Rainbow->setFont(font1);
        radioButton_Rainbow->setChecked(true);

        verticalLayout->addWidget(radioButton_Rainbow);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);


        verticalLayout_2->addWidget(groupBox_ColorMode);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        pushButton_load = new QPushButton(centralwidget);
        pushButton_load->setObjectName(QString::fromUtf8("pushButton_load"));
        pushButton_load->setMinimumSize(QSize(50, 40));

        horizontalLayout->addWidget(pushButton_load);

        pushButton_save = new QPushButton(centralwidget);
        pushButton_save->setObjectName(QString::fromUtf8("pushButton_save"));
        pushButton_save->setMinimumSize(QSize(50, 40));

        horizontalLayout->addWidget(pushButton_save);


        verticalLayout_2->addLayout(horizontalLayout);


        horizontalLayout_3->addLayout(verticalLayout_2);

        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(50);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy2);
        qvtkWidget->setMinimumSize(QSize(640, 480));

        horizontalLayout_3->addWidget(qvtkWidget);

        PCLViewer->setCentralWidget(centralwidget);
        QWidget::setTabOrder(pushButton_load, radioButton_z);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0, QApplication::UnicodeUTF8));
        groupBox_Axis->setTitle(QApplication::translate("PCLViewer", "Color on axis", 0, QApplication::UnicodeUTF8));
        radioButton_x->setText(QApplication::translate("PCLViewer", "X", 0, QApplication::UnicodeUTF8));
        radioButton_y->setText(QApplication::translate("PCLViewer", "Y", 0, QApplication::UnicodeUTF8));
        radioButton_z->setText(QApplication::translate("PCLViewer", "Z", 0, QApplication::UnicodeUTF8));
        groupBox_ColorMode->setTitle(QApplication::translate("PCLViewer", "Color mode", 0, QApplication::UnicodeUTF8));
        radioButton_BlueRed->setText(QApplication::translate("PCLViewer", "Blue to red", 0, QApplication::UnicodeUTF8));
        radioButton_GreenMagenta->setText(QApplication::translate("PCLViewer", "Green to magenta", 0, QApplication::UnicodeUTF8));
        radioButton_WhiteRed->setText(QApplication::translate("PCLViewer", "White to red", 0, QApplication::UnicodeUTF8));
        radioButton_GreyRed->setText(QApplication::translate("PCLViewer", "Grey / red", 0, QApplication::UnicodeUTF8));
        radioButton_Rainbow->setText(QApplication::translate("PCLViewer", "Rainbow", 0, QApplication::UnicodeUTF8));
        pushButton_load->setText(QApplication::translate("PCLViewer", "Load file", 0, QApplication::UnicodeUTF8));
        pushButton_save->setText(QApplication::translate("PCLViewer", "Save file", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
