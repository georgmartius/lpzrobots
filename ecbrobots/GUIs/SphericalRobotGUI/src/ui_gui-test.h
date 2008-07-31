/********************************************************************************
** Form generated from reading ui file 'gui-test.ui'
**
** Created: Mi Jul 30 04:45:22 2008
**      by: Qt User Interface Compiler version 4.3.1
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_GUI_2D_TEST_H
#define UI_GUI_2D_TEST_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGroupBox>
#include <QtGui/QWidget>

class Ui_SphericalRobotGUI
{
public:
    QGroupBox *groupBox;
    QGroupBox *groupBox_2;
    QGroupBox *groupBox_3;
    QDialogButtonBox *buttonBox;

    void setupUi(QWidget *SphericalRobotGUI)
    {
    if (SphericalRobotGUI->objectName().isEmpty())
        SphericalRobotGUI->setObjectName(QString::fromUtf8("SphericalRobotGUI"));
    SphericalRobotGUI->resize(400, 300);
    groupBox = new QGroupBox(SphericalRobotGUI);
    groupBox->setObjectName(QString::fromUtf8("groupBox"));
    groupBox->setGeometry(QRect(10, 10, 120, 80));
    groupBox_2 = new QGroupBox(SphericalRobotGUI);
    groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
    groupBox_2->setGeometry(QRect(140, 10, 120, 80));
    groupBox_3 = new QGroupBox(SphericalRobotGUI);
    groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
    groupBox_3->setGeometry(QRect(270, 10, 120, 80));
    buttonBox = new QDialogButtonBox(SphericalRobotGUI);
    buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
    buttonBox->setGeometry(QRect(230, 270, 160, 23));
    buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::NoButton|QDialogButtonBox::Ok);

    retranslateUi(SphericalRobotGUI);

    QMetaObject::connectSlotsByName(SphericalRobotGUI);
    } // setupUi

    void retranslateUi(QWidget *SphericalRobotGUI)
    {
    SphericalRobotGUI->setWindowTitle(QApplication::translate("SphericalRobotGUI", "Form", 0, QApplication::UnicodeUTF8));
    groupBox->setTitle(QApplication::translate("SphericalRobotGUI", "GroupBox", 0, QApplication::UnicodeUTF8));
    groupBox_2->setTitle(QApplication::translate("SphericalRobotGUI", "GroupBox", 0, QApplication::UnicodeUTF8));
    groupBox_3->setTitle(QApplication::translate("SphericalRobotGUI", "GroupBox", 0, QApplication::UnicodeUTF8));
    Q_UNUSED(SphericalRobotGUI);
    } // retranslateUi

};

namespace Ui {
    class SphericalRobotGUI: public Ui_SphericalRobotGUI {};
} // namespace Ui

#endif // UI_GUI_2D_TEST_H
