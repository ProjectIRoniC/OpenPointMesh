/*
*/

// Macroguard
#ifndef _filesystemhelper
#define _filesystemhelper

namespace vba
{
	namespace filesystemhelper
	{
		/// <summary>
		/// Creates a name for an output file
		/// </summary>
		std::string getOutputFileName(const std::string outputDirectory, const std::string inputFile, const std::string fileExtension);

		/// <summary>
		/// Creates a directory
		/// </summary>
		bool createDirectory(const std::string directoryPath);

		/// <summary>
		/// Deletes a directory and all its contents if it exists
		/// </summary>
		bool deleteDirectory(const std::string directoryPath);
	}
}

// End Macroguard
#endif
