#ifndef JNI_INCLUDE_INVERTEDFILE_H_
#define JNI_INCLUDE_INVERTEDFILE_H_

#include <map>

#include "vocab_tree/VLImage.h"

namespace EyeMARS {

class InvertedFile {
 public:
  std::map<VLImage*, int> images;
  void insert(VLImage* img);
  int occurances(VLImage* img);
  void print();

  /////////////////////////////
  // additional function declaration to save VT
  void writetofile(std::ofstream& InvertedList);

};

}  // End of namespace: eyemars */

#endif  // JNI_INCLUDE_INVERTEDFILE_H_
