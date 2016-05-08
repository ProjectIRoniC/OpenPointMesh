/*
*/

#include <boost/filesystem.hpp>
#include <iostream>
#include "filesystemHelper.h"

/// <summary>
/// Creates a name for an output file
/// </summary>
std::string vba::filesystemhelper::getOutputFileName(const std::string outputDirectory, const std::string inputFile, const std::string fileExtension)
{
	// If the path contains '/' characters
	if (inputFile.find_last_of('/') != std::string::npos)
	{
		// Check if the directory needs a trailing '/'
		if (outputDirectory[outputDirectory.size()] == '/' || outputDirectory[outputDirectory.size()] == '\\')
			return std::string(outputDirectory + inputFile.substr(inputFile.find_last_of('/') + 1) + fileExtension);

		return std::string(outputDirectory + std::string("/") + inputFile.substr(inputFile.find_last_of('/') + 1) + fileExtension);
	}
	// If the path contains '\' characters
	else if (inputFile.find_last_of('\\') != std::string::npos)
	{
		// Check if the directory needs a trailing '\'
		if (outputDirectory[outputDirectory.size()] == '/' || outputDirectory[outputDirectory.size()] == '\\')
			return std::string(outputDirectory + inputFile.substr(inputFile.find_last_of('\\') + 1) + fileExtension);

		return std::string(outputDirectory + std::string("\\") + inputFile.substr(inputFile.find_last_of('\\') + 1) + fileExtension);
	}

	// Otherwise the input file does not contain a path
	// Check if the directory needs a trailing '/'
	if (outputDirectory[outputDirectory.size()] == '/' || outputDirectory[outputDirectory.size()] == '\\')
		return std::string(outputDirectory + inputFile + fileExtension);

	return std::string(outputDirectory + std::string("/") + inputFile + fileExtension);
}

/// <summary>
/// Creates a directory
/// </summary>
bool vba::filesystemhelper::createDirectory(const std::string directoryPath)
{
	boost::system::error_code returnedError;
	if (!boost::filesystem::exists(directoryPath))
		boost::filesystem::create_directories(directoryPath, returnedError);

	if (returnedError)
	{
		std::cerr << "\nUnable to create directory:" << returnedError.category().name() << '\n' << returnedError.value() << '\n';
		return false;
	}

	return true;
}

/// <summary>
/// Deletes a directory and all its contents if it exists
/// </summary>
bool vba::filesystemhelper::deleteDirectory(const std::string directoryPath)
{
	boost::system::error_code returnedError;
	if (boost::filesystem::exists(directoryPath))
		boost::filesystem::remove_all(directoryPath, returnedError);

	if (returnedError)
	{
		std::cerr << "\nUnable to delete directory:" << returnedError.category().name() << '\n' << returnedError.value() << '\n';
		return false;
	}

	return true;
}
