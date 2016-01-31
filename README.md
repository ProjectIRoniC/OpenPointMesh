# OpenPointMesh
University of Colorado Denver, Computer Science, Senior Design, ProjectIrOniC Team, 2016

-----

## Developer Environment Setup
### Ubuntu 64-bit
#### Prerequisites
##### Point Cloud Library 1.7 for Ubuntu
1. Navigate to http://www.pointclouds.org/downloads/linux.html
2. Follow the instructions to install the PCL prebuilt binaries for your OS

##### Qt 4.8
1. Navigate to http://www.qt.io/download/
2. Follow the instructions to download the Qt manager
3. Run the installer and select Qt 4.8

##### C++ 4.8.4
1. Run the following command in terminal: `sudo apt-get install g++`

#### Getting and Compiling the source code (Enter all commands in terminal)
1. `sudo apt-get install git` (follow the prompts to install git)
2. `git clone git@github.com:ProjectIRoniC/OpenPointMesh.git`
3. `cd OpenPointMesh`
4. `mkdir build`
5. `cd build`
6. `cmake ..`
7. `make`

-----

### OSX 10.11 El Capitan
#### Prerequisites
##### Point Cloud Library 1.7 for Mac
1. Install a Java JDK (required for Homebrew only) from the Java website http://www.oracle.com/technetwork/java/javase/downloads/index.html
2. Open a Terminal window
3. Install Homebrew with the command `ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`
4. Press enter when prompted
5. Enter your admin password when prompted
6. Enter the command `brew update`
7. Enter the command `brew tap homebrew/science`
8. Enter the command `brew install pcl --with-openni2` (this can take a long time to complete)

#### Getting and Compiling the source code (Enter all commands in terminal)
1. `sudo apt-get install git` (follow the prompts to install git)
2. `git clone git@github.com:ProjectIRoniC/OpenPointMesh.git`
3. `cd OpenPointMesh`
4. `mkdir build`
5. `cd build`
6. `cmake ..`
7. `make`
