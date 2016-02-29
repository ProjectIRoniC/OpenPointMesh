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

	void setAccuracyValue( QMainWindow* mainwindow , unsigned int value );

public slots:
	void accept();
	void reject();

private:
	Ui::AccuracyControlMenu ui;
	QMainWindow* main_window;

	unsigned int accuracy_value;

};

#endif // LIGHTEDITMENU_H
