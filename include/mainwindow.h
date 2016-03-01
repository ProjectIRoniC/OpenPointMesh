#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <QPlainTextEdit>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread.hpp>
#include "AccuracyControlMenu.h"


class QAction;
class QActionGroup;
class QLabel;
class QMenu;

namespace Ui {
class MainWindow;
}
//class MainWindow;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    /*
     * Constructor
     */

    /* Post: Initialize GUI object */
    explicit MainWindow( QWidget *parent = 0 );

    /*
     * Destructor
     */
    ~MainWindow();


private:
    /*
     * Pre:  Msg contains a non null string to be output to GUI display
     * Post: Msg has been added to GUI display.
     * Note: Currently does not support is_error effect
     */
    void appendMessageToOutputBuffer( std::string msg, const bool is_error = false );

    /*
     * Pre:  outputBuffer has been initialized to a non-null object.
     * Post: If outputBuffer is non-empty then element is send to appenedMessage
     */
    void processOutputQueue();

    /* Pre:  All buttons are either enabled(true) or enabled(false).
     * Post: Browse_oni    = enabled
     *       Browse_output = disabled
     *       Start         = disabled
     *       Close         = enabled
     */
    void setInitialButtonState() ;

    /*
     * Post: Browse_oni    = disabled
     *       Browse_output = disabled
     *       Start         = disabled
     */
    void setButtonsAllDisabledState();

    /*
     * Slots
     */
private slots:

    /*
     * Pre:  oniFileName points to an oni file. outpuFolderName points to dir where output will be placed
     * Post: Begins oni to mesh transformation
     */
    void on_Start_clicked();
    /*
     * Pre:  constrollerConstant must from the controller constants set
     * Post: nextstep for given constant is executed.
     * Note: This is where software flow is defined. Connect signal to a slot and call nextStep at the end of
     *       every subtask passing the controller constant for the next action to be performed
     */
    void nextStep( const int& controllerConstant );
    /*
     * Post: Opens files explorer so that the user can select an output folder. outputFolderName stores absolute path
     */
    void on_Browse_output_clicked();
    /*
     * Post: Display concole is moved to the cursor position
     * Note: Not sure if this is needed.
     */
    void ensureCursorVisible( QString );

    /*
     * Menu functions
     */
    /*
     * Post: Opens file explorer to choose .oni file.
     *       Path is stored in onifileName
     */
    void openSlot();
    /*
     * Post: Closes application. Destroys GUI
     */
    void exitSlot();

    /*
     * Post: Opens a new window for the user to input a sample rate.
     *       Must be a number. Rules for this number are defined within
     *       oni-to-pcd class.
     */
    void sampleFrameRateSlot();

    /*
     * Not Implemented
     */
    void omitFramesSlot();
    void filterAccuracySlot();
    void meshAccuracySlot();
    void aboutSlot();
    void viewWikiSlot();



    void on_oni_browse_button_clicked();

    void onAccuracyControlDialogClose();

signals:
    /*
     * Pre:  msg is non-nul / non-empty.
     * Post: AppendMessage has been called with msg. EnsureCursorvisibile has also been called.
     */
    void appendToConsole( QString msg );


    /*
     * Program Flow
     */
    /*
     * Start is not really needed. Start is for readability. Start signal is attached to nextstep.
     * Post: nextStep has been called
     */
    void start( int );
    /*
     * Signal that marks the end of onitToPcd conversion.
     * Post: nextStep is called
     */
    void oniToPCDFinished( int );
    /*
     * Signal that marks the end of cloudstiching.
     * Post: nextstep is called
     */
    void cloudStitcherFinished( int );

    /*
     * Sginal that marks the end of the mesh construction
     * Post: nextstep is called with passed int
     */
    void meshConstructorFinished( int );



private:

    /*
     * Program Flow Controller Constants
     * These constants are used to control the flow of the program. These contants are used and
     * should be managed inside nextstep.
     */
    static const int ONITOPCD = 0;
    static const int CLOUDSTITCHER = 1;
    static const int MESHCONSTRUCTOR = 2;
    static const int FINISHED = 3;

    /*
     * Main ui
     */
    Ui::MainWindow *ui;
    /*
     * Contains onifilenames with absolute paths
     */
    QStringList oniFileNames;
    /*
     * Contains user desired output folder absolute path
     */
    QString outputFolderName;
    /*
     * buffer that contains all strings to be displayed to user
     */
    boost::lockfree::spsc_queue<std::string>* outputBuffer;
    /*
     * worker thread for processing and controlling outputbuffer
     */
    boost::thread* outputMessageThread;
    /*
     * worker thread that will do all the tasks that user enters
     */
    boost::thread* taskThread;


    unsigned int accuracy_control_value;

    /*
     * Actions for menue
     */
    QAction *openAct;
    QAction *exitAct;
    QAction *omitFramesAct;
    QAction *filterAccuracyAct;
    QAction *meshAccuracyAct;
    QAction *sampleRateAct;
    QAction *aboutAct;
    QAction *viewWikiAct;

    /*
     * Menu
     */
    QMenu *fileMenu;
    QMenu *settingMenu;
    QMenu *helpMenu;


    /*
     * Dialog Boxes
     */
     AccuracyControlMenu* accuracyControlMenu;

    /*
     * Marks the completion of the last step of controller constants
     * Used for processing outputbuffer.
     */
    bool done;
    /*
     * Pre:  outputBuffer is non-null
     * Post: outputbuffer elements are being removed and entered into display concole
     */
    void checkOutputBuffer();
    /*
     * Controller for ONI to pcd module
     * Post: oni file has been converted to many pcd file
     */
    void oniToPCDController();
    /*
     * Controller for multiple pcd file to single point cloud file
     * Post: final pointcloud file has been generated from pcd files
     */
    void cloudStitcherController();
    /*
     * Conroller for point cloud file to mesh
     * Post: point cloud file has been converted to single mesh file
     */
    void meshConstructorController();
    /*
     * Stops and deletes task thread.
     */
    void clearTaskThread();


};

#endif // MAINWINDOW_H
