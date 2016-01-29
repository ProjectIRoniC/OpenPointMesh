#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include "../include/mainwindow.h"
#include "../build/ui_mainwindow.h"
#include "../include/oni-to-pcd.h"
#include "../include/CloudStitcher.h"
#include "../include/filesystemHelper.h"
#include "../include/MeshConstructor.h"

MainWindow::MainWindow( QWidget *parent ) :
    QMainWindow( parent ),
    ui( new Ui::MainWindow )
{
    ui->setupUi( this );
    ui->plainTextEdit->setStyleSheet( "QLabel {background-color : white; }" );
    ui->plainTextEdit->setReadOnly( true );
    ui->plainTextEdit->setCenterOnScroll( true );
    ui->plainTextEdit->ensureCursorVisible();
    this->outputBuffer = new boost::lockfree::spsc_queue<std::string>( 200 );
    this->done = false;

    outputMessageThread = new boost::thread( &MainWindow::processOutputQueue, this );

    // process the order in which we want tasks to be run
    connect( this, SIGNAL(appendToConsole(QString)), ui->plainTextEdit, SLOT(insertPlainText(QString)) );
    connect( this, SIGNAL(start(int)), this, SLOT(nextStep(int)) );
    connect( this, SIGNAL(oniToPCDFinished(int)), this, SLOT(nextStep(int)) );
    connect( this, SIGNAL(appendToConsole(QString)), this, SLOT(ensureCursorVisible(QString)) );
    connect( this, SIGNAL(cloudStitcherFinished(int)), this, SLOT(nextStep(int)) );
    outputFolderName = QString( "" );
    oniFileNames = QStringList();
    taskThread = NULL;
}

MainWindow::~MainWindow()
{
    delete outputBuffer;
    delete outputMessageThread;
    delete taskThread;
    delete ui;
    // TODO add cleanup for threads
}

void MainWindow::ensureCursorVisible( QString s )
{
    ui->plainTextEdit->ensureCursorVisible();
}

void MainWindow::nextStep( const int& step )
{
    switch( step ) {

    case ONITOPCD:
        clearTaskThread();
        taskThread = new boost::thread( &MainWindow::oniToPCDController, this );
        break;
    case CLOUDSTITCHER :
        clearTaskThread();
        taskThread = new boost::thread( &MainWindow::cloudStitcherController, this );
        break;
    case MESHCONSTRUCTOR:
        clearTaskThread();
        taskThread = new boost::thread( &MainWindow::meshConstructorController, this );
        break;
    default :
        QString errmsg = "MESSAGE: Well this is embarassing, there seems to be an ";
        errmsg += "error with my instruction set and I not quiet sure how to fix it. ";
        errmsg += "I affraid I cannot continue processing. Please try again later.";
        appendToConsole( errmsg );
        appendToConsole( QString("ERROR: Program execution stopped.") );
        break;
    }

}

void MainWindow::clearTaskThread()
{
    if( taskThread == NULL )
        return;

    taskThread->join(); // this should return immeditly as the thread should already have finished if this function is called.
    delete taskThread;
    taskThread = NULL;
}

void MainWindow::meshConstructorController()
{
	appendMessage( "Start mesh constructor..." );
    vba::MeshConstructor* mMeshConstructor = new vba::MeshConstructor();

    // pass outputbuffer
    mMeshConstructor->setOutputBuffer( this->outputBuffer );
    //just pass in a string with the path to the stitched pcd file
    mMeshConstructor->setInputFilename( vba::filesystemhelper::getOutputFileName(this->outputFolderName.toStdString(), oniFileNames[0].toStdString(), "_finalPointCloud.pcd") );
	appendMessage( "File used for construction " + mMeshConstructor->getInputFilename() );
    //pass in a string of the output path where the final mesh should be sent. The file does not have to exist already.
    //The second parameter lets you choose the output filetype. I would just stick with PLY for now.
    mMeshConstructor->setOutputFilename( this->outputFolderName.toStdString() + "/finished_mesh.ply" , vba::PLY );
    appendMessage( "Output final mesh to " + mMeshConstructor->getOutputFilename() );

    //this does the rest
    mMeshConstructor->constructMesh();

	appendMessage( "Mesh constructor complete." );
    delete mMeshConstructor;
    // emit meshConstructorFinished();
}

void MainWindow::cloudStitcherController()
{
	appendMessage( "Start point cloud stitching..." );
    vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;
	mCloudStitcher->setOutputBuffer( this->outputBuffer );
    
	// Loop through each input file
	for( unsigned h = 0; h < oniFileNames.size(); ++h )
	{
		appendMessage( "Working on stitching " + oniFileNames[h].toStdString() + "...\n" );
		mCloudStitcher->setOutputPath( vba::filesystemhelper::getOutputFileName(this->outputFolderName.toStdString(), oniFileNames[h].toStdString(), "_finalPointCloud.pcd") );
		appendMessage( "Output final point cloud to " + mCloudStitcher->getOutputPath() );
		mCloudStitcher->stitchPCDFiles( vba::filesystemhelper::getOutputFileName(this->outputFolderName.toStdString(), oniFileNames[h].toStdString(), "") );
	}
    
    delete mCloudStitcher;
    emit cloudStitcherFinished( MESHCONSTRUCTOR );
}

void MainWindow::on_Browse_clicked()
{
    QStringList files = QFileDialog::getOpenFileNames(
                this,
                "Select one or more files to open",
                "/home",
                "Text files (*.oni)");
    if( files.size() > 0 )
    {
        oniFileNames = files;
		
		for( unsigned j = 0; j < oniFileNames.size(); ++j )
			appendMessage( oniFileNames[j].toStdString() + " selected" );
    }
    else
    {
        appendMessage( "No .ONI file selected" );
    }
}

void MainWindow::on_Cancel_clicked()
{
    this->close();
}

void MainWindow::processOutputQueue()
{
    std::string temp = "";
    boost::posix_time::seconds waitTime( 1 );
    while( !done || !this->outputBuffer->empty() )
    {
        if( this->outputBuffer->empty() )
        {
            boost::this_thread::sleep( waitTime );
        }
        else
        {
            if( this->outputBuffer->pop(temp) )
            {
                QString output = QString::fromStdString( temp );
                emit appendToConsole( output );
                temp = ""; // clear value for safety
            }
        }
    }
}


void MainWindow::oniToPCDController()
{
    if( outputFolderName == "" )
	{
		appendMessage( "No output directory selected. Please select an output folder where you would like the final oni to go." );
        return;
    }

    if( oniFileNames.isEmpty() )
	{
		appendMessage( "ERROR: Please browse for an .ONI file before clicking start." );
        return;
    }
	
	appendMessage( "Start oni data output..." );
	appendMessage( "Output to " + outputFolderName.toStdString() );
	unsigned frameSkipMod = 25;
	vba::OniToPcd* oniReader = new vba::OniToPcd(outputFolderName.toStdString(), frameSkipMod, this->outputBuffer);
	
	// Loop through each input file
	for( unsigned i = 0; i < oniFileNames.size(); ++i )
	{
		appendMessage( "Working on file " + oniFileNames[i].toStdString() );
		oniReader->outputOniData( oniFileNames[i].toStdString() );
	}
	
    emit oniToPCDFinished( CLOUDSTITCHER );

}

void MainWindow::on_Start_clicked()
{
    emit start( ONITOPCD );
}


void MainWindow::on_radioButton_toggled( bool checked )
{
    ui->plainTextEdit->setVisible( checked );
    ui->plainTextEdit->ensureCursorVisible();
}


// ** Helper Functions ** //
void MainWindow::appendMessage( std::string msg,const bool is_error )
{
    QString output = QString::fromStdString( msg );
    ui->plainTextEdit->appendPlainText( output );
}
void MainWindow::on_Browse_output_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    "/home",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);

    if( dir.size() > 0 )
    {
        outputFolderName = QString::fromStdString( boost::filesystem::absolute(dir.toStdString()).string() );
        appendMessage( outputFolderName.toStdString() + " selected for output", false );
    }
    else
    {
        appendMessage( "No outputFolder Selected file selected", false );
    }
}
