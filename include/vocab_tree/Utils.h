#ifndef JNI_INCLUDE_UTILS_H_
#define JNI_INCLUDE_UTILS_H_

/**

@file Utils.h
@brief Utilities for testing the VocabTree
@author Elliot Branson

**/


#include <sys/stat.h>
#include <sys/types.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <time.h>
#include <boost/range.hpp>
#include <sys/time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include <vector>
#include <algorithm>

#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Common.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/VLImage.h"


// A struct for keeping track of file positions and corresponding line indices
struct FilePos {
  std::streampos pos;
  size_t line;
  FilePos() : pos(0), line(0) { }
};


/**
 ** @brief Makes a directory, including necessary parent directories,
 **			with no error if it already exists.
 **
 ** @param s 	The directory name.
 ** @param mode	the permissions on the new directory.
 **
 ** Identical to the bash command mkdir -p
 **/
int mkpath(const char *s, mode_t mode);

/**
 ** @brief Initializes a timer such that calling ::toc() returns the time since
 ** 	::tic() was last called.
 **/
void tic();

/**
 ** @brief Calculates time in milliseconds since ::tic() was last called.
 **
 ** @return the time.
 **/
int64 toc();


#endif  // JNI_INCLUDE_UTILS_H_
