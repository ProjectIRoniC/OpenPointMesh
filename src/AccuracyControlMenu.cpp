#include "AccuracyControlMenu.h"

AccuracyControlMenu::AccuracyControlMenu(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi( this );

	this->accuracy_value = 5;

}

AccuracyControlMenu::~AccuracyControlMenu()
{

}

void AccuracyControlMenu::setAccuracyValue( QMainWindow* mainwindow , unsigned int value )
{
    this->main_window = mainwindow;
    this->accuracy_value = value;

    ui.accuracy_slider.setValue( value );
}


void AccuracyControlMenu::accept()
{
    this->main_window->setAccuracyControlValue( ui.accuracy_slider.getValue() );
    this->accuracy_value = ui.accuracy_slider.getValue();
}

void AccuracyControlMenu::reject()
{
    this->accuracy_value = 5;
}


