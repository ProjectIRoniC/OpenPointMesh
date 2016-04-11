#!/bin/bash

now=$(date +"%m_%d_%Y")

suffix=".xml"
cloudstitch="xml:../reports/CloudStitcherTest_"
meshconstruct="xml:../reports/MeshConstructorTest_"
onitopcd="xml:../reports/OniToPcdTest_"
filesys="xml:../reports/FilesystemHelperTest_"

test1=$cloudstitch$now$suffix
test2=$meshconstruct$now$suffix
test3=$onitopcd$now$suffix
test4=$filesys$now$suffix



rm -rf reports
mkdir reports

cd ./build/

echo "Building Tests....."
make

echo "Running All Tests......"

./CloudStitcherTest --gtest_output=$test1 > /dev/null 2>&1
echo "	Finished Cloud Stitcher Test"

./MeshConstructorTest --gtest_output=$test2 > /dev/null 2>&1
echo "	Finished Mesh Constructor Test"

./OniToPcdTest --gtest_output=$test3 > /dev/null 2>&1
echo "	Finished Oni to Pcd Test"

./FilesystemHelperTest --gtest_output=$test4 > /dev/null 2>&1
echo "	Finished Filesystem Helper Test"
echo "Finished All Tests"

cd ../
