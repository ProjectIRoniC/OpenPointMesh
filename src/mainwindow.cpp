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
    connect( this, SIGNAL(meshConstructorFinished(int)), this, SLOT(nextStep(int)) );

    // set initial values for varibles
    outputFolderName = QString( "" );
    oniFileNames = QStringList();
    taskThread = NULL;

    /* Add a horizontal Scroll Bar*/
    this->ui->plainTextEdit->setLineWrapMode(QPlainTextEdit::NoWrap);

    /* create actions for menu */
    /* Open */
    openAct = new QAction(tr("&Open"), this);
    openAct->setShortcuts(QKeySequence::New);
    openAct->setStatusTip(tr("Create a new file"));
    connect(openAct, SIGNAL(triggered()), this, SLOT(openSlot()));

    /* Exit */
    exitAct = new QAction(tr("&Exit"), this);
    exitAct->setShortcuts(QKeySequence::New);
    connect(exitAct, SIGNAL(triggered()), this, SLOT(exitSlot()));

    /* Omit Frames */
    omitFramesAct = new QAction(tr("&Omit Frames"), this);
    omitFramesAct->setShortcuts(QKeySequence::New);
    omitFramesAct->setStatusTip(tr("Select frames to omit with gui"));
    connect(omitFramesAct, SIGNAL(triggered()), this, SLOT(omitFramesSlot()));

    /* Filter Accuracy */
    filterAccuracyAct = new QAction(tr("&Filter Accuracy"), this);
    filterAccuracyAct->setShortcuts(QKeySequence::New);
    filterAccuracyAct->setStatusTip(tr("Selecting accuracy of the program effects run time"));
    connect(filterAccuracyAct, SIGNAL(triggered()), this, SLOT(filterAccuracySlot()));

    /* Mesh Accuracy */
    meshAccuracyAct = new QAction(tr("&Mesh Accuracy"), this);
    meshAccuracyAct->setShortcuts(QKeySequence::New);
    meshAccuracyAct->setStatusTip(tr("Select Mesh Creation Accuracy"));
    connect(meshAccuracyAct, SIGNAL(triggered()), this, SLOT(meshAccuracySlot()));

    /* About */
    aboutAct = new QAction(tr("&About"), this);
    aboutAct->setShortcuts(QKeySequence::New);
    aboutAct->setStatusTip(tr("About"));
    // connect(aboutAct, SIGNAL(triggered()), qApp, SLOT(aboutSlot()));
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(aboutSlot()));

    /* View Wiki Page */
    viewWikiAct = new QAction(tr("&Wiki"), this);
    viewWikiAct->setShortcuts(QKeySequence::New);
    viewWikiAct->setStatusTip(tr("Wiki Page"));
    connect(viewWikiAct, SIGNAL(triggered()), this, SLOT(viewWikiSlot()));

    /* create menu */
    fileMenu = menuBar()->addMenu(tr("&File"));
    settingMenu = menuBar()->addMenu(tr("&Settings"));
    helpMenu = menuBar()->addMenu(tr("&Help"));

    /* add action to menu */
    fileMenu->addAction(openAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    settingMenu->addAction(omitFramesAct);
    settingMenu->addAction(filterAccuracyAct);
    settingMenu->addAction(meshAccuracyAct);
    settingMenu->addSeparator();

    helpMenu->addAction(aboutAct);
    helpMenu->addAction(viewWikiAct);
    helpMenu->addSeparator();

    // Set color of Main Window
    //this->ui->
    //this->ui->radioButton->setStyleSheet("background-color: blue ");
    //emit appendToConsole( output );
    // Set initial button state
    setInitialButtonState();
}

/*
 * ***************************Action button Controls***************************
 */

void MainWindow::openSlot()
{
    QStringList files = QFileDialog::getOpenFileNames(
                this,
                "Select one or more files to open",
                "/home",
                "Text files (*.oni)");
    if( files.size() > 0 )
    {
        oniFileNames = files;

        for( int j = 0; j < oniFileNames.size(); ++j )
            appendMessageToOutputBuffer( oniFileNames[j].toStdString() + " selected\n" );
        // update buttons
        this->ui->Browse_output->setEnabled(true);

    }
    else
    {
        appendMessageToOutputBuffer( "No .ONI file selected\n" );
    }
}

void MainWindow::exitSlot()
{
    this->close();
}

void MainWindow::omitFramesSlot()
{
    std::cout << "inside omitframesslot\n";
}

void MainWindow::filterAccuracySlot()
{
    std::cout << "inside filteraccuracyslot\n";
}

void MainWindow::meshAccuracySlot()
{
    std::cout << "inside mesh accuracy slot\n";
}

void MainWindow::aboutSlot()
{
    QMessageBox::about(this, tr("About Menu"),
            tr("This <b>messagebox</b> example shows how we can create "
               "an about page or instructions."));
}

void MainWindow::viewWikiSlot()
{
    std::cout << "inside view wiki slot\n";
}

/*
 * ***************************Action button Controls***************************
 */
void MainWindow::setInitialButtonState()
{
    this->ui->Browse_output->setEnabled(false);
    this->ui->Start->setEnabled(false);
}

void MainWindow::setButtonsAllDisabledState()
{
    this->ui->Browse_output->setEnabled(false);
    this->ui->Start->setEnabled(false);
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
        setButtonsAllDisabledState();
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
    case FINISHED:
        clearTaskThread();
        setInitialButtonState();
        break;
    default :
        setInitialButtonState();
        QString errmsg = "MESSAGE: Well this is embarassing, there seems to be an ";
        errmsg += "error with my instruction set and I not quiet sure how to fix it. ";
        errmsg += "I affraid I cannot continue processing. Please try again later.\n";
        appendToConsole( errmsg );
        appendToConsole( QString("ERROR: Program execution stopped.\n") );
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
    appendMessageToOutputBuffer( "Start mesh constructor...\n" );
    vba::MeshConstructor* mMeshConstructor = new vba::MeshConstructor();

    // pass outputbuffer
    mMeshConstructor->setOutputBuffer( this->outputBuffer );
    //just pass in a string with the path to the stitched pcd file
    mMeshConstructor->setInputFilename( vba::filesystemhelper::getOutputFileName(this->outputFolderName.toStdString(), oniFileNames[0].toStdString(), "_finalPointCloud.pcd") );
    appendMessageToOutputBuffer( "File used for construction " + mMeshConstructor->getInputFilename()  + '\n');
    //pass in a string of the output path where the final mesh should be sent. The file does not have to exist already.
    //The second parameter lets you choose the output filetype. I would just stick with PLY for now.
    mMeshConstructor->setOutputFilename( this->outputFolderName.toStdString() + "/finished_mesh.ply" , vba::PLY );
    appendMessageToOutputBuffer( "Output final mesh to " + mMeshConstructor->getOutputFilename() + '\n');

    //this does the rest
    mMeshConstructor->constructMesh();

    appendMessageToOutputBuffer( "Mesh constructor complete.\n" );
    delete mMeshConstructor;
    emit meshConstructorFinished(FINISHED);
}

void MainWindow::cloudStitcherController()
{
    appendMessageToOutputBuffer( "Start point cloud stitching...\n" );
    vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;
	mCloudStitcher->setOutputBuffer( this->outputBuffer );
    
	// Loop through each input file
	for( int h = 0; h < oniFileNames.size(); ++h )
	{
        appendMessageToOutputBuffer( "Working on stitching " + oniFileNames[h].toStdString() + "...\n" );
		mCloudStitcher->setOutputPath( vba::filesystemhelper::getOutputFileName(this->outputFolderName.toStdString(), oniFileNames[h].toStdString(), "_finalPointCloud.pcd") );
        appendMessageToOutputBuffer( "Output final point cloud to " + mCloudStitcher->getOutputPath() + '\n');
		mCloudStitcher->stitchPCDFiles( vba::filesystemhelper::getOutputFileName(this->outputFolderName.toStdString(), oniFileNames[h].toStdString(), "") );
	}
    
    delete mCloudStitcher;
    emit cloudStitcherFinished( MESHCONSTRUCTOR );
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
        appendMessageToOutputBuffer( "No output directory selected. Please select an output folder where you would like the final oni to go.\n" );
        return;
    }

    if( oniFileNames.isEmpty() )
	{
        appendMessageToOutputBuffer( "ERROR: Please browse for an .ONI file before clicking start.\n" );
        return;
    }
	
    appendMessageToOutputBuffer( "Start oni data output...\n" );
    appendMessageToOutputBuffer( "Output to " + outputFolderName.toStdString() + '\n' );
	unsigned frameSkipMod = 25;
	vba::OniToPcd* oniReader = new vba::OniToPcd(outputFolderName.toStdString(), frameSkipMod, this->outputBuffer);
	
	// Loop through each input file
	for( int i = 0; i < oniFileNames.size(); ++i )
	{
        appendMessageToOutputBuffer( "Working on file " + oniFileNames[i].toStdString() + '\n');
		oniReader->outputOniData( oniFileNames[i].toStdString() );
	}
	
    emit oniToPCDFinished( CLOUDSTITCHER );

}


// Button functions

void MainWindow::on_Browse_oni_clicked()
{
    QStringList files = QFileDialog::getOpenFileNames(
                this,
                "Select one or more files to open",
                "/home",
                "Text files (*.oni)");
    if( files.size() > 0 )
    {
        oniFileNames = files;

        for( int j = 0; j < oniFileNames.size(); ++j )
            appendMessageToOutputBuffer( oniFileNames[j].toStdString() + " selected\n" );
        // update buttons
        this->ui->Browse_output->setEnabled(true);

    }
    else
    {
        appendMessageToOutputBuffer( "No .ONI file selected\n" );
    }
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
        appendMessageToOutputBuffer( outputFolderName.toStdString() + " selected for output\n", false );
        this->ui->Browse_output->setEnabled(false);
        this->ui->Start->setEnabled(true);
    }
    else
    {
        appendMessageToOutputBuffer( "No outputFolder Selected file selected\n", false );
    }
}

void MainWindow::on_Cancel_clicked()
{
    this->close();
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
void MainWindow::appendMessageToOutputBuffer( std::string msg,const bool is_error )
{
    QString output = QString::fromStdString( msg );
    this->outputBuffer->push(msg);

}

