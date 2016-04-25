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
#include "../include/AccuracyControlMenu.h"
#include "../include/AboutDialog.h"
#include "../qtest/include/debugger.h"
#include "../include/OPM_NiViewer.h"
#include <QMessageBox>
#include <QInputDialog>


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

    // label that displays 'File'
    this->ui->file_display_label->setFrameStyle(QFrame::Sunken);
    this->ui->file_display_label->setAlignment(Qt::AlignCenter);

    // textbrowser that displays files that will be selected by the user
    this->ui->textBrowser->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    this->ui->textBrowser->setAlignment(Qt::AlignLeft);


    outputMessageThread = new boost::thread( &MainWindow::processOutputQueue, this );

    // process the order in which we want tasks to be run
    connect( this, SIGNAL(appendToConsole(QString)), ui->plainTextEdit, SLOT(insertPlainText(QString)) );
    connect( this, SIGNAL(start(int)), this, SLOT(nextStep(int)) );
    connect( this, SIGNAL(oniToPCDFinished(int)), this, SLOT(nextStep(int)) );
    connect( this, SIGNAL(appendToConsole(QString)), this, SLOT(ensureCursorVisible(QString)) );
    connect( this, SIGNAL(cloudStitcherFinished(int)), this, SLOT(nextStep(int)) );
    connect( this, SIGNAL(meshConstructorFinished(int)), this, SLOT(nextStep(int)) );
    connect( this, SIGNAL(omitFramesFinished(int)), this , SLOT(nextStep(int)) );

    // set initial values for varibles
    outputFolderName = QString( "" );
    oniFileNames = QStringList();
    taskThread = NULL;

    /* Add a horizontal Scroll Bar*/
    this->ui->plainTextEdit->setLineWrapMode(QPlainTextEdit::NoWrap);

    /* create actions for menu */
    /* Open */
    openAct = new QAction(tr("&Open"), this);
    openAct->setShortcuts(QKeySequence::Open);
    openAct->setStatusTip(tr("Open a .ONI file"));
    connect(openAct, SIGNAL(triggered()), this, SLOT(openSlot()));
    openAct->setObjectName("openAct");

    /* Exit */
    exitAct = new QAction(tr("&Exit"), this);
    exitAct->setShortcuts(QKeySequence::Close);
    exitAct->setStatusTip(tr("Exit the program"));
    connect(exitAct, SIGNAL(triggered()), this, SLOT(exitSlot()));
    exitAct->setObjectName("exitAct");

    /* Omit Frames */
    omitFramesAct = new QAction(tr("&Omit Frames"), this);
    omitFramesAct->setShortcuts(QKeySequence::Find);
    omitFramesAct->setStatusTip(tr("Select frames to omit"));
    omitFramesAct->setCheckable(true);
    connect(omitFramesAct, SIGNAL(triggered()), this, SLOT(omitFramesSlot()));
    omitFramesAct->setObjectName("omitFramesAct");

    /* Filter Accuracy */
    filterAccuracyAct = new QAction(tr("&Filter Accuracy"), this);
    filterAccuracyAct->setShortcuts(QKeySequence::New);
    filterAccuracyAct->setStatusTip(tr("Selecting accuracy of the program may effects run time"));
    connect(filterAccuracyAct, SIGNAL(triggered()), this, SLOT(filterAccuracySlot()));
    filterAccuracyAct->setObjectName("filterAccuracyAct");

    /* Change the samplerate for an oni video*/
    sampleRateAct = new QAction(tr("&Oni Sample Rate"), this);
    sampleRateAct->setShortcuts(QKeySequence::Save);
    sampleRateAct->setStatusTip(tr("Changes sample frame rate (per second)"));
    connect(sampleRateAct, SIGNAL(triggered()), this, SLOT(sampleFrameRateSlot()));
    sampleRateAct->setObjectName("sampleRateAct");

    /* About */
    aboutAct = new QAction(tr("&About"), this);
    aboutAct->setShortcuts(QKeySequence::SelectAll);
    aboutAct->setStatusTip(tr("About"));
    // connect(aboutAct, SIGNAL(triggered()), qApp, SLOT(aboutSlot()));
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(aboutSlot()));
    aboutAct->setObjectName("aboutAct");

    /* View Wiki Page */
    viewWikiAct = new QAction(tr("&Wiki"), this);
    viewWikiAct->setShortcuts(QKeySequence::Underline);
    viewWikiAct->setStatusTip(tr("Wiki Page"));
    connect(viewWikiAct, SIGNAL(triggered()), this, SLOT(viewWikiSlot()));
    viewWikiAct->setObjectName("viewWikiAct");

    meshOutputOBJAct = new QAction( tr( "&OBJ") , this );
    meshOutputPLYAct = new QAction( tr( "&PLY") , this );
    meshOutputVTKAct = new QAction( tr( "&VTK") , this );
    meshOutputFiletypeAct = new QActionGroup( this );

    meshOutputOBJAct->setObjectName("meshOutputOBJAct");
    meshOutputPLYAct->setObjectName("meshOutputPLYAct");
    meshOutputVTKAct->setObjectName("meshOutputVTKAct");

    meshOutputFiletypeAct->addAction( meshOutputOBJAct );
    meshOutputFiletypeAct->addAction( meshOutputPLYAct );
    meshOutputFiletypeAct->addAction( meshOutputVTKAct );

    meshOutputOBJAct->setCheckable( true );
    meshOutputPLYAct->setCheckable( true );
    meshOutputVTKAct->setCheckable( true );
    meshOutputPLYAct->setChecked( true );

    /* create menu */
    fileMenu = menuBar()->addMenu(tr("&File"));
    settingMenu = menuBar()->addMenu(tr("&Settings"));
    helpMenu = menuBar()->addMenu(tr("&Help"));

    /* add action to menu */
    fileMenu->addAction(openAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);
    fileMenu->setObjectName("fileMenu");

    settingMenu->addAction(omitFramesAct);
    settingMenu->addAction(sampleRateAct);
    settingMenu->addAction(filterAccuracyAct);
    settingMenu->addSeparator();

    meshOutputSubMenu = settingMenu->addMenu( "Mesh Output Filetype" );
    //meshOutputSubMenu->addAction( meshOutputFiletypeAct );
    meshOutputSubMenu->addAction( meshOutputOBJAct );
    meshOutputSubMenu->addAction( meshOutputPLYAct );
    meshOutputSubMenu->addAction( meshOutputVTKAct );
    connect( meshOutputOBJAct , SIGNAL( triggered() ) , this , SLOT( meshOutputOBJSlot() ));
    connect( meshOutputOBJAct , SIGNAL( triggered() ) , this , SLOT( meshOutputPLYSlot() ));
    connect( meshOutputOBJAct , SIGNAL( triggered() ) , this , SLOT( meshOutputVTKSlot() ));

    helpMenu->addAction(aboutAct);
    helpMenu->addAction(viewWikiAct);
    helpMenu->addSeparator();

    accuracyControlMenu = new AccuracyControlMenu( this );
    connect( accuracyControlMenu , SIGNAL( accepted() ) , this , SLOT( onAccuracyControlDialogClose() ));

    aboutDialog = new AboutDialog( this );

    // Set initial button state
    setInitialButtonState();

    this->accuracy_control_value = 5;
    this->mesh_filetype = vba::PLY;

    this->samplingRate = vba::DEFAULT_FRAME_SKIP;
    this->hasSamplingRate = false;

}



/*
 * ***************************Action button Controls***************************
 */

void MainWindow::openSlot()
{
    on_oni_browse_button_clicked();
}

void MainWindow::exitSlot()
{
    this->close();
}

void MainWindow::omitFramesSlot()
{
    /* omitFrameAct retains the check status */
    /* send message to output */
    if(omitFramesAct->isChecked()) {
        appendMessageToOutputBuffer("" + omitFramesAct->iconText().toStdString() + " enabled\n");
    }
    else {
        appendMessageToOutputBuffer("" + omitFramesAct->iconText().toStdString() + " disabled\n");
    }
}

void MainWindow::filterAccuracySlot()
{
    accuracyControlMenu->setAccuracyValue( this->accuracy_control_value );
    accuracyControlMenu->show();
}

void MainWindow::meshOutputOBJSlot()
{
    this->mesh_filetype = vba::OBJ;

}

void MainWindow::meshOutputPLYSlot()
{
    this->mesh_filetype = vba::PLY;

}

void MainWindow::meshOutputVTKSlot()
{
    this->mesh_filetype = vba::VTK;
}

void MainWindow::sampleFrameRateSlot()
{
    do {
        hasSamplingRate = false;
        samplingRate = QInputDialog::getInt(this,
                                        tr("QInputDialog::getInt()"),
                                        "label",
                                        samplingRate,
                                        vba::DEFAULT_FRAME_SKIP,
                                        vba::MAX_FRAME_SKIP,
                                        1,
                                        &hasSamplingRate);

        if (hasSamplingRate && vba::OniToPcd::minimumSamplingRate(samplingRate))
        {
            return;
        }
    } while( hasSamplingRate && !vba::OniToPcd::minimumSamplingRate(samplingRate));
}

void MainWindow::aboutSlot()
{
    aboutDialog->show();
}

void MainWindow::viewWikiSlot()
{
    QDesktopServices::openUrl( QUrl( "https://github.com/ProjectIRoniC/OpenPointMesh/wiki" ));
}

void MainWindow::onAccuracyControlDialogClose()
{
    this->accuracy_control_value = this->accuracyControlMenu->getAccuracyValue();
}

/*
 * ***************************Action button Controls***************************
 */
void MainWindow::setInitialButtonState()
{
    this->ui->Browse_output->setEnabled(false);
    this->ui->Start->setEnabled(false);
    this->ui->oni_browse_button->setEnabled(true);
}

void MainWindow::setButtonsAllDisabledState()
{
    this->ui->Browse_output->setEnabled(false);
    this->ui->Start->setEnabled(false);
}

MainWindow::~MainWindow()
{
    done = true;
    delete outputBuffer;
    delete outputMessageThread;
    delete taskThread;
    delete ui;

    delete accuracyControlMenu;
    delete aboutDialog;
    // TODO add cleanup for threads
}

void MainWindow::ensureCursorVisible( QString s )
{
    ui->plainTextEdit->ensureCursorVisible();
}

void MainWindow::nextStep( const int& step )
{
    switch( step ) {
    case OMITFRAMES:
        setButtonsAllDisabledState();
        clearTaskThread();
        taskThread = new boost::thread( &MainWindow::omitFramesController, this );
        break;
    case ONITOPCD:
        workingOnFile = true;
        setButtonsAllDisabledState();
        clearTaskThread();
        taskThread = new boost::thread( &MainWindow::oniToPCDController, this );
        break;
    case CLOUDSTITCHER :
        workingOnFile = true;
        clearTaskThread();
        taskThread = new boost::thread( &MainWindow::cloudStitcherController, this );
        break;
    case MESHCONSTRUCTOR:
        workingOnFile = true;
        clearTaskThread();
        taskThread = new boost::thread( &MainWindow::meshConstructorController, this );
        break;
    case FINISHED:
        workingOnFile = false;
        clearTaskThread();
        setInitialButtonState();
        break;
    default :
        workingOnFile = false;
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
    if( this->mesh_filetype == vba::OBJ )
    {
        mMeshConstructor->setOutputFilename( this->outputFolderName.toStdString() + "/finished_mesh.obj" , this->mesh_filetype );
    }

    if( this->mesh_filetype == vba::PLY )
    {
        mMeshConstructor->setOutputFilename( this->outputFolderName.toStdString() + "/finished_mesh.ply" , this->mesh_filetype );
    }

    if( this->mesh_filetype == vba::VTK )
    {
        mMeshConstructor->setOutputFilename( this->outputFolderName.toStdString() + "/finished_mesh.vtk" , this->mesh_filetype );
    }

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
	mCloudStitcher->setFilterResolution( this->accuracy_control_value );

	// Loop through each input file
	for( int h = 0; h < oniFileNames.size(); ++h )
	{
        appendMessageToOutputBuffer( "Working on stitching " + oniFileNames[h].toStdString() + "...\n" );
		mCloudStitcher->setOutputPath( vba::filesystemhelper::getOutputFileName(this->outputFolderName.toStdString(), oniFileNames[h].toStdString(), "_finalPointCloud") );
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
    unsigned frameSkipMod = (hasSamplingRate) ? samplingRate : 25;
	vba::OniToPcd* oniReader = new vba::OniToPcd(outputFolderName.toStdString(), frameSkipMod, this->outputBuffer);

	// Loop through each input file and its associated set of omitted frames, which may be an empty set
	for( int i = 0; i < oniFileNames.size(); ++i )
	{
        appendMessageToOutputBuffer( "Working on file " + oniFileNames[i].toStdString() + '\n');
		oniReader->outputOniData( oniFileNames[i].toStdString() );
        oniReader->setOmmittedFrames( this->ommittedFrames[i] );
	}

    emit oniToPCDFinished( CLOUDSTITCHER );

}



void MainWindow::on_Browse_output_clicked()
{
    QString dir;
    if(debugger::QTESTING && !outputFolderName.isEmpty()) {
            dir = outputFolderName;
    }
    else {
        dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                        "/home",
                                                        QFileDialog::ShowDirsOnly
                                                        | QFileDialog::DontResolveSymlinks);
    }

    if( dir.size() > 0 )
    {
        outputFolderName = QString::fromStdString( boost::filesystem::absolute(dir.toStdString()).string() );
        appendMessageToOutputBuffer( outputFolderName.toStdString() + " selected for output\n", false );
        this->ui->Start->setEnabled(true);
    }
    else
    {
        appendMessageToOutputBuffer( "No outputFolder Selected file selected\n", false );
    }
}


void MainWindow::on_Start_clicked()
{
    emit start( OMITFRAMES );
}

// ** Helper Functions ** //
void MainWindow::appendMessageToOutputBuffer( std::string msg,const bool is_error )
{
    QString output = QString::fromStdString( msg );
    this->outputBuffer->push(msg);
}


void MainWindow::on_oni_browse_button_clicked()
{
    QStringList files;
    if(debugger::QTESTING && !ui->textBrowser->toPlainText().isEmpty()) {
            files << ui->textBrowser->toPlainText().split(";", QString::SkipEmptyParts);
    }
    else {
        files = QFileDialog::getOpenFileNames(
                    this,
                    "Select one or more files to open",
                    "/home",
                    "Text files (*.oni)");
    }

    if( files.size() > 0 )
    {
        oniFileNames = files;
        this->ui->textBrowser->setText("");
        for( int j = 0; j < oniFileNames.size(); ++j ) {
            // appendMessageToOutputBuffer( oniFileNames[j].toStdString() + " selected\n" );
            this->ui->textBrowser->insertPlainText(oniFileNames[j] + ' ');
            this->ui->textBrowser->insertHtml("<b>;</b> ");
            this->ui->textBrowser->insertPlainText(" ");
        }
        // update buttons
        this->ui->Browse_output->setEnabled(true);

    }
    else
    {
        appendMessageToOutputBuffer( "No .ONI file selected\n" );
    }
}

void MainWindow::omitFramesController()
{
    int of[] = {3, 4, 5, 6, 7, 8, 9, 10, 20, 21, 22, 23, 24, 25, 30, 32, 33, 34, 35, 36, 37, 39};
    this->ommittedFrames.push_back(std::set<int> (of, of+22));

    emit omitFramesFinished( ONITOPCD );
}

/* Setters and Getters */

void MainWindow::setTextBrowser(QString txt)
{
    ui->textBrowser->setText(txt);
}
QString MainWindow::getTextBrowser()
{
    return ui->textBrowser->toPlainText();
}

void MainWindow::setOutputFolderName(QString txt)
{
    outputFolderName = txt;
}

QString MainWindow::getOutputFolderName() {
    return outputFolderName;
}

bool MainWindow::hasStartedWorkingOnFile()
{
    return workingOnFile;
}
