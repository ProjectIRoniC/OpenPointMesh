#include "../include/mainwindow.h"
#include "../build/ui_mainwindow.h"
#include "../include/oni-to-pcd.h"
#include <iostream>
#include "../include/CloudStitcher.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->plainTextEdit->setStyleSheet("QLabel {background-color : white; }");
    ui->plainTextEdit->setReadOnly(true);
    ui->plainTextEdit->setCenterOnScroll(true);
    ui->plainTextEdit->ensureCursorVisible();
    this->outputBuffer = new boost::lockfree::spsc_queue<std::string>(200);
    this->done = false;

    outputMessageThread = new boost::thread(&MainWindow::processOutputQueue, this);

    // process the order in which we want tasks to be run
    connect(this, SIGNAL(appendToConsole(QString)), ui->plainTextEdit, SLOT(insertPlainText(QString)));
    connect(this, SIGNAL(start(int)), this, SLOT(nextStep(int)));
    connect(this, SIGNAL(oniToPCDFinished(int)), this, SLOT(nextStep(int)));
    connect(this, SIGNAL(appendToConsole(QString)), this, SLOT(ensureCursorVisible(QString)));
    connect(this, SIGNAL(cloudStitcherFinished(int)), this, SLOT(nextStep(int)));
    outputFolderName = "";
    oniFileName = "";
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

void MainWindow::ensureCursorVisible(QString s){
    ui->plainTextEdit->ensureCursorVisible();
    return;
}

void MainWindow::nextStep(const int& step) {

    switch(step) {

    case ONITOPCD:
        clearTaskThread();
        taskThread = new boost::thread(&MainWindow::oniToPCDController, this);
        break;
    case CLOUDSTITCHER :
        clearTaskThread();
        taskThread = new boost::thread(&MainWindow::cloudStitcherController, this);
        break;
    case MESHCONSTRUCTOR:
        clearTaskThread();
        taskThread = new boost::thread(&MainWindow::meshConstructorController, this);
        break;
    default :
        QString errmsg = "MESSAGE: Well this is embarassing, there seems to be an ";
        errmsg += "error with my instruction set and I not quiet sure how to fix it. ";
        errmsg += "I affraid I cannot continue processing. Please try again later.";
        appendToConsole(errmsg);
        appendToConsole(QString("ERROR: Program execution stopped."));
        break;
    }

}

void MainWindow::clearTaskThread() {
    if(taskThread == NULL) {
        return;
    }
    taskThread->join(); // this should return immeditly as the thread should already have finished if this function is called.
    delete taskThread;
    taskThread = NULL;
}

void MainWindow::meshConstructorController() {
    vba::MeshConstructor* mMeshConstructor = new vba::MeshConstructor();

    // pass outputbuffer
    mMeshConstructor->setOutputBuffer(this->outputBuffer);
    //just pass in a string with the path to the stitched pcd file
    mMeshConstructor->setInputFilename(this->outputFolderName.toStdString() + "/finalPointCloud.pcd" );

    //pass in a string of the output path where the final mesh should be sent. The file does not have to exist already.
    //The second parameter lets you choose the output filetype. I would just stick with PLY for now.
    mMeshConstructor->setOutputFilename( this->outputFolderName.toStdString() + "/finished_mesh.ply" , vba::PLY );

    //this does the rest
    mMeshConstructor->constructMesh();

    delete mMeshConstructor;
    // emit meshConstructorFinished();
}

void MainWindow::cloudStitcherController() {


    /*
     *  if(oniFileName == "") {
        appendMessage("ERROR: Please browse for an .ONI file before clicking start");
        return;
    }
     */
    vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;

    std::string pcdFilesToStitchDir(this->outputFolderName.toStdString() + "/pcdTemp");
    std::string stitchedOniOutputDir (this->outputFolderName.toStdString() + "/finalPointCloud.pcd");

    mCloudStitcher->setOutputPath( stitchedOniOutputDir );

    mCloudStitcher->setOutputBuffer(this->outputBuffer);

    mCloudStitcher->stitchPCDFiles( pcdFilesToStitchDir );

    delete mCloudStitcher;
    emit cloudStitcherFinished(MESHCONSTRUCTOR);
    return;
}

void MainWindow::on_Browse_clicked()
{
    QStringList files = QFileDialog::getOpenFileNames(
                this,
                "Select one or more files to open",
                "/home",
                "Text files (*.oni)");
    if(files.size() > 0) {
        oniFileName = files[0];
        appendMessage(oniFileName.toStdString() + " selected" );
    }
    else {
        appendMessage("No .ONI file selected");
    }
}

void MainWindow::on_Cancel_clicked()
{
    this->close();
}

void MainWindow::processOutputQueue(){
    std::string temp = "";
    boost::posix_time::seconds waitTime(5);
    while(!done || !this->outputBuffer->empty()){
        if(this->outputBuffer->empty()){
            boost::this_thread::sleep(waitTime);
        }
        else {
            if(this->outputBuffer->pop(temp)) {
                QString output = QString::fromStdString(temp);
                emit appendToConsole(output);
                temp = ""; // clear value for safety
            }
        }
    }
    return;
}


void MainWindow::oniToPCDController(){
    /*
     argv[2] contains path to output pcdfiles
     dir contains where the pcdfiles will actually be output
     output contains where the final pointcloud file will be stored off of argv[2]
    */
    if(outputFolderName == ""){
        this->outputBuffer->push("No output directory selected. Please select an output folder where you would like the final oni to go.");
        return;
    }

    if(oniFileName == "") {
        this->outputBuffer->push("ERROR: Please browse for an .ONI file before clicking start");
        return;
    }
    // setup for oni-many-pcd files
    int argc = 3;
    char* argv[3];
    int length = strlen(oniFileName.toStdString().c_str());
    argv[1] = new char[length + 1]();
    strncpy(argv[1], oniFileName.toStdString().c_str(), length+1);

    length = strlen(outputFolderName.toStdString().c_str());
    argv[2] = new char[length + 1]();
    strncpy(argv[2], outputFolderName.toStdString().c_str(), length+1);

    std::cout <<argv[1] << "-"; // '-' shows ending characters
    std::cout << "\n" << argv[2] << "-";
    vba::oni2pcd::setOutputBuffer(this->outputBuffer);
    vba::oni2pcd::driver(argc, argv);
    emit oniToPCDFinished(CLOUDSTITCHER);

}

void MainWindow::on_Start_clicked()
{
    emit start(ONITOPCD);
}


void MainWindow::on_radioButton_toggled(bool checked)
{
    ui->plainTextEdit->setVisible(checked);
    ui->plainTextEdit->ensureCursorVisible();
}


// ** Helper Functions ** //
void MainWindow::appendMessage(std::string msg,const bool is_error) {
    QString output = QString::fromStdString(msg);
    ui->plainTextEdit->appendPlainText(output);
}
void MainWindow::on_Browse_output_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    "/home",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);

    if(dir.size() > 0) {
        outputFolderName = dir;
        appendMessage(outputFolderName.toStdString() + " selected for output\n", false);
    }
    else {
        appendMessage("No outputFolder Selected file selected\n", false);
    }
}
