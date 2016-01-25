#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <string>
#include <QPlainTextEdit>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/atomic.hpp>
#include <boost/lockfree/policies.hpp>
#include <boost/thread.hpp>
#include <../include/MeshConstructor.h>

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
    explicit MainWindow(QWidget *parent = 0);

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
    void appendMessage(std::string msg, const bool is_error = false);

    /*
     * Pre:  outputBuffer has been initialized to a non-null object.
     * Post: If outputBuffer is non-empty then element is send to appenedMessage
     */
    void processOutputQueue();

    /*
     * Slots
     */
private slots:

    /*
     * Post: Opens file explorer to choose .oni file. 
     *       Path is stored in onifileName
     */
    void on_Browse_clicked();
    /*
     * Post: Closes application. Destroys GUI
     */
    void on_Cancel_clicked();
    /*
     * Pre:  oniFileName points to an oni file. outpuFolderName points to dir where output will be placed
     * Post: Begins oni to mesh transformation
     */
    void on_Start_clicked();
    /*
     * Post: display console visibility is toggled
     */
    void on_radioButton_toggled(bool checked);
    /*
     * Pre:  constrollerConstant must from the controller constants set
     * Post: nextstep for given constant is executed.
     * Note: This is where software flow is defined. Connect signal to a slot and call nextStep at the end of
     *       every subtask passing the controller constant for the next action to be performed
     */
    void nextStep(const int& controllerConstant);
    /*
     * Post: Opens files explorer so that the user can select an output folder. outputFolderName stores absolute path
     */
    void on_Browse_output_clicked();
    /*
     * Post: Display concole is moved to the cursor position
     * Note: Not sure if this is needed.
     */
    void ensureCursorVisible(QString);

signals:
    /*
     * Pre:  msg is non-nul / non-empty.
     * Post: AppendMessage has been called with msg. EnsureCursorvisibile has also been called.
     */
    void appendToConsole(QString msg);


    /*
     * Program Flow
     */
    /*
     * Start is not really needed. Start is for readability. Start signal is attached to nextstep.
     * Post: nextStep has been called
     */
    void start(int);
    /*
     * Signal that marks the end of onitToPcd conversion.
     * Post: nextStep is called
     */
    void oniToPCDFinished(int);
    /*
     * Signal that marks the end of cloudstiching.
     * Post: nextstep is called
     */
    void cloudStitcherFinished(int);


private:

    /*
     * Program Flow Controller Constants
     * These constants are used to control the flow of the program. These contants are used and
     * should be managed inside nextstep.
     */
    static const int ONITOPCD = 0;
    static const int CLOUDSTITCHER = 1;
    static const int MESHCONSTRUCTOR = 2;

    /*
     * Main ui
     */
    Ui::MainWindow *ui;
    /*
     * Contains onifilename absolute path
     */
    QString oniFileName;
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
