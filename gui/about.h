/********************************************************************************
** Form generated from reading UI file 'about.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef ABOUT_H
#define ABOUT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QTextBrowser>
#include "QtWidgets/qformlayout.h"

QT_BEGIN_NAMESPACE

class Ui_AboutGui
{
public:
    QTextBrowser *textBrowser;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_client_verson;
    QLabel *label_firmware_version;
    QGroupBox *groupBox;

    void setupUi(QDialog *AboutGui)
    {
        if (AboutGui->objectName().isEmpty())
            AboutGui->setObjectName(QStringLiteral("AboutGui"));
        AboutGui->resize(700, 493);
        QVBoxLayout* layout = new QVBoxLayout();
        QFormLayout* layout_form = new QFormLayout();


        label = new QLabel();
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 20, 91, 31));
        QFont font;
        font.setPointSize(10);
        label->setFont(font);
        label_2 = new QLabel();
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(20, 60, 81, 31));
        label_2->setFont(font);
        label_client_verson = new QLabel();
        label_client_verson->setObjectName(QStringLiteral("label_client_verson"));
        label_client_verson->setGeometry(QRect(110, 24, 571, 21));
        label_client_verson->setFont(font);
        label_firmware_version = new QLabel();
        label_firmware_version->setObjectName(QStringLiteral("label_firmware_version"));
        label_firmware_version->setGeometry(QRect(110, 65, 571, 20));
        label_firmware_version->setFont(font);

        layout_form->addRow(label, label_client_verson);
        layout_form->addRow(label_2, label_firmware_version);
        layout_form->setVerticalSpacing(16);

        QVBoxLayout* layout_txt = new QVBoxLayout();
        groupBox = new QGroupBox();
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 120, 680, 362));
        groupBox->setFont(font);
        groupBox->setLayout(layout_txt);

        textBrowser = new QTextBrowser();
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(19, 140, 662, 331));

        layout_txt->addWidget(textBrowser);
        layout->addLayout(layout_form);
        layout->addWidget(groupBox);
        layout->setSpacing(16);
        AboutGui->setLayout(layout);

        retranslateUi(AboutGui);

        QMetaObject::connectSlotsByName(AboutGui);
    } // setupUi

    void retranslateUi(QDialog *AboutGui)
    {
        AboutGui->setWindowTitle(QApplication::translate("AboutGui", "\345\205\263\344\272\216", Q_NULLPTR));
        label->setText(QApplication::translate("AboutGui", "\345\256\242\346\210\267\347\253\257\347\211\210\346\234\254\357\274\232", Q_NULLPTR));
        label_2->setText(QApplication::translate("AboutGui", "\345\233\272\344\273\266\347\211\210\346\234\254\357\274\232", Q_NULLPTR));
        label_client_verson->setText(QString());
        label_firmware_version->setText(QString());
        groupBox->setTitle(QApplication::translate("AboutGui", "\344\272\247\345\223\201\350\257\246\346\203\205\357\274\232", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class AboutGui: public Ui_AboutGui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // ABOUT_H
