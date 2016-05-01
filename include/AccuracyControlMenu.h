#ifndef ACCURACYCONTROLMENU_H
#define ACCURACYCONTROLMENU_H

#include <QDialog>
#include <QMainWindow>
#include "ui_AccuracyControlMenu.h"


class AccuracyControlMenu : public QDialog
{
	Q_OBJECT

public:
	AccuracyControlMenu(QWidget *parent);
	~AccuracyControlMenu();

	void setAccuracyValue( unsigned int value );
	unsigned int getAccuracyValue() {return this->accuracy_value;}

public slots:
	void accept();

private:
	Ui::AccuracyControlMenu ui;

	unsigned int accuracy_value;

};

#endif // LIGHTEDITMENU_H
