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

void AccuracyControlMenu::setAccuracyValue( unsigned int value )
{
    this->accuracy_value = value;

    ui.accuracy_slider->setSliderPosition( value );
}


void AccuracyControlMenu::accept()
{
    this->accuracy_value = ui.accuracy_slider->sliderPosition();

    this->close();
    emit accepted();
}



