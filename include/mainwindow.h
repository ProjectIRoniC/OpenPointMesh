/*
 * @file    mainwindow.h
 * @brief   This is the main gui window for Mesh Generation. At least (1) .oni file is required to run
 *          through the program.
 * @bug     Opening multiple windows has not been fully test.
 * @bug     Closing the window after mesh construction has already begun but has not be finished may create zombie process.
 *
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <QPlainTextEdit>
#include <QDesktopServices>
#include <QUrl>
#include <QActionGroup>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread.hpp>
#include "AccuracyControlMenu.h"
#include "AboutDialog.h"
#include "MeshConstructor.h"
#include <set>
#include <vector>

class QAction;
class QActionGroup;
class QLabel;
class QMenu;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    /*
    @brief a brief description
    @details a more detailed description
    @pre any preconditions if they exist
    @param any parameters used if they exist
    @return expected return values
    */

    /*
    @brief      Constructor for mainwindow class
    @details    Initializes signals and slots for gui.
    @pre        None
    @return     GUI has been initialized
    */
    explicit MainWindow( QWidget *parent = 0 );

    /*
        @brief      Destructor mainwindow object
        @details    QT library will delete its own dynamic objects. Use this to remove non QT objects that you create.
        @return     mainwindow object has been destoryed
    */
    ~MainWindow();

    void setTextBrowser(QString txt);

    QString getTextBrowser ();

    void setOutputFolderName(QString txt);

    QString getOutputFolderName();

    bool hasStartedWorkingOnFile();

    //// @brief reads a file containing omitted frames
    std::set<int> readOmittedFramesFile ( char* filename );

private:
    /*
     * @brief       Adds a message to be displayed as output on the details panel
     * @details     Adds msg to outputBuffer
     * @pre         outputBuffer has been initialized
     * @param:      msg contains the the string that will be added to outputBuffer
     * @param:      is_error - flag that indicates if the message should be treated normally or like an error
     * @bug         currently does not support is_error
     * @return      Msg has been added to GUI outputBuffer
     */
    void appendMessageToOutputBuffer( std::string msg, const bool is_error = false );

    /*
     * Pre:  outputBuffer has been initialized to a non-null object.
     * Post: If outputBuffer is non-empty then element is send to appenedMessage
     */
    /*
     * @brief       Adds msg to outputBuffer
     * @details     Polls the outputbuffer to see if there is something to be added to details panel
     * @pre         outputBuffer has been initialized to a non-null object.
     * @return      If outputBuffer is non-empty then element is send to appenedMessage
     */
    void processOutputQueue();

    /*
     * @brief       Sets initial button state
     * @pre         All buttons in GUI have been initliazed
     * @return      Browse_oni    = enabled
     *              Browse_output = disabled
     *              Start         = disabled
     *              Close         = enabled
     */
    void setInitialButtonState() ;

    /*
     * @brief       Sets initial button state
     * @pre         All buttons in GUI have been initliazed
     * @return      Browse_oni    = disabled
     *              Browse_output = disabled
     *              Start         = disabled
     */
    void setButtonsAllDisabledState();

    /*
     * Slots
     */
private slots:

    /*
     * @pre         oniFileName points to an oni file. outpuFolderName points to dir where output will be placed
     * @return      Begins oni to mesh transformation
     */
    void on_Start_clicked();

    /*
     * @pre:        constrollerConstant must from the controller constants set
     * @return:     nextstep for given constant is executed.
     * @brief:      This is where software flow is defined. Connect signal to a slot and call nextStep at the end of
     *              every subtask passing the controller constant for the next action to be performed
     */
    void nextStep( const int& controllerConstant );
    /*
     * @return: Opens files explorer so that the user can select an output folder. outputFolderName stores absolute path
     */
    void on_Browse_output_clicked();
    /*
     * @brief       Slot that controls where the cursor is on the display panel
     * @param       QString is a placerholder object. Connected signals and slots require the same parameters. Currently depends on appendToConsole signal
     * @return      Display concole is moved to the cursor position
     * Note: Not sure if this is needed.
     */
    void ensureCursorVisible( QString );

    /*
     * Menu functions
     */

    /*
     * @brief       Slot needs to be connected to a signal to use
     * @return:     Opens file explorer to choose .oni file. Path is stored in onifileName
     */
    void openSlot();

    /*
     * @brief       Slot needs to be connected to a signal to use
     * @return: Closes application. Destroys GUI
     */
    void exitSlot();

    /*
     * @brief       Slot needs to be connected to a signal to use
     * @pre         Must be a number. Rules for this number are defined within
     *              oni-to-pcd class.
     * @return      Opens a new window for the user to input a sample rate.
     *
     */
    void sampleFrameRateSlot();

    /*
     * @brief       Slot needs to be connected to a signal to use
     * @pre         aboutDialog class contains information to be presented to user when selected
     * @return      Opens aboutdialog class.
     *
     */
    void aboutSlot();
    /*
     * @brief       Slot needs to be connected to a signal to use
     * @pre         URL for wiki page online must exist. URL defined within function
     * @return      Opens default browser to specified url.
     *
     */
    void viewWikiSlot();

    /*
        * @brief       Slot needs to be connected to a signal to use
        * @pre         NONE
        * @return      Opens file explorer and shows .off files only.
        *
        */
       void omitFramesSlot();


    /*
     * Not Implemented
     */
    void filterAccuracySlot();

    void meshOutputOBJSlot();
    void meshOutputPLYSlot();
    void meshOutputVTKSlot();


    void on_oni_browse_button_clicked();

    void onAccuracyControlDialogClose();

signals:
    /*
     * @brief       Signals are emitted
     * @pre:        msg is non-nul / non-empty.
     * @return:     AppendMessage has been called with msg. EnsureCursorvisibile has also been called.
     */
    void appendToConsole( QString msg );

    /*
     * Program Flow
     */

    /*
     * @details     Start is not really needed. Start is for readability. Start signal is attached to nextstep.
     * @return      nextStep has been called
     */
    void start( int );
    /*
     * Sginal that marks the end of the omit frames component
     * Post: nextstep is called with passed int
     */
    void omitFramesFinished(int);
    /*
     * @brief     Signal that marks the end of onitToPcd conversion.
     * @return    nextStep is called
     */
    void oniToPCDFinished( int );
    /*
     * @brief       Signal that marks the end of cloudstiching.
     * @return      nextstep is called
     */
    void cloudStitcherFinished( int );

    /*
     * @brief       Sginal that marks the end of the mesh construction
     * @return      nextstep is called with passed int
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
    static const int OMITFRAMES = 3;
    static const int FINISHED = 4;

    std::set<int> omittedFrames;

    /*
     * if the user has changed the sampleing rate
     */
    bool hasSamplingRate;
    int samplingRate;
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
    /*
     * Stores the .off file name if user has selected it.
     */
    QString omitFileName;


    unsigned int accuracy_control_value;

    //holds the filetype that the mesh will be outputted to
    vba::MESH_FILETYPE mesh_filetype;

    /*
     * Actions for menue
     */
    QAction *openAct;
    QAction *exitAct;
    QAction *omitFramesAct;
    QAction *filterAccuracyAct;
    QAction *sampleRateAct;
    QAction *aboutAct;
    QAction *viewWikiAct;

    QActionGroup *meshOutputFiletypeAct;
    QAction *meshOutputPLYAct;
    QAction *meshOutputOBJAct;
    QAction *meshOutputVTKAct;

    /*
     * Menu
     */
    QMenu *fileMenu;
    QMenu *settingMenu;
    QMenu *helpMenu;
    QMenu *meshOutputSubMenu;


    /*
     * Dialog Boxes
     */
     AccuracyControlMenu* accuracyControlMenu;
     AboutDialog* aboutDialog;

    /*
     * Marks the completion of the last step of controller constants
     * Used for processing outputbuffer.
     * TODO : check where this is being monitored. May be obselete
     */
    bool done;

    /* True when the start button has been pressed and through all processes until final
     * output file is created */
    bool workingOnFile;

    /*
     * @Pre:        outputBuffer is non-null
     * @return:     outputbuffer elements are being removed and entered into display concole
     */
    void checkOutputBuffer();
    /*
     * @brief       Controller for ONI to pcd module
     * @return      oni file has been converted to many pcd file
     */
    void oniToPCDController();
    /*
     * @brief       Controller for multiple pcd file to single point cloud file
     * @return      final pointcloud file has been generated from pcd files
     */
    void cloudStitcherController();
    /*
     * @brief       Conroller for point cloud file to mesh
     * @return      point cloud file has been converted to single mesh file
     */
    void meshConstructorController();

    /*
     * Controller for omit frames component.
     * Pre:  oniFileNames contains a list of oni files.
     *       a viewer will be presented to the user so that they may select what
     *       frames to omit
     * Post: the viewer has been called with each file in oniFileNames
     */
    void omitFramesController();

    /*
     * @brief       Stops and deletes task thread.
     * @return      task thread has been stopped and deallocated
     */
    void clearTaskThread();

};

#endif // MAINWINDOW_H
