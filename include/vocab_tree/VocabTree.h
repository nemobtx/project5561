#ifndef JNI_INCLUDE_VOCABTREE_H_
#define JNI_INCLUDE_VOCABTREE_H_

#include <stdlib.h>
#include <unordered_set>

#include <map>
#include <deque>
#include <vector>
#include <string>

#include "vocab_tree/VTFeature.h"
#include "vocab_tree/KMeans.h"
#include "vocab_tree/VLImage.h"
#include "vocab_tree/InvertedFile.h"
#include "vocab_tree/VTFeatureLoader.h"

#include "Eigen/Sparse"

/** @namespace eyemars
 * The eyemars namespace.
 */
namespace EyeMARS {

/** A class which impliments D. Nister's Vocabulary Tree for Loop Closure
 *
 *  The VocabTree class provices an implimentation of D. Nister's Vocab Tree
 *  as described in [Nister, 2006]. This class both builds the tree and allows
 *  for place Re-recognition.
 */
class VocabTree {
 public:

  // Maximum depth of the Vocab Tree
  unsigned int LEVELS;
  // Branching factor of the Vocab Tree
  unsigned int branching_factor;
  // Centers which are the nodes in the Vocab Tree
  std::vector<VTFeature*> centers;
  // Marks whether center[i] is valid
  std::vector<unsigned int> valid;
  // The relative enthropy weights
  std::map<int, double> weights;
  // A collection of indexes and InvertedFiles
  std::map<int, InvertedFile*> InvertedFiles;
  // Ratio used for image matching
  float ratio;


  VocabTree(unsigned int levels = 10, unsigned int branching_factor = 8) :
    LEVELS(levels), branching_factor(branching_factor) {ratio = 0.05;}

  // Builds the Vocab Tree using the input parameter
  void build(std::vector<VLImage*>& images,
             unsigned int orientation_split = 0);
  // quantizes the image based on the VTFeatures of the image
  void quantize(VLImage* query);
  // Returns the index of the leaf which is closest to input from centers vector
  int leaf(VTFeature* query);
  // Inserts the VLImage into the tree using the method leaf
  void insert(VLImage* query);

  // Find the best k matching images
  void match(VLImage*, int k, std::vector<VLImage*> &);
  // Find the matches which score less than threshold
  void match(VLImage*, float threshold, std::vector<VLImage*> &);
  // match which impliments both threshold and k based matching
  void match(VLImage*, float threshold, int k, std::vector<VLImage*> &);

  // Inserts the VLImage and finds the best k matches
  void matchInsert(VLImage*, int k, std::vector<VLImage*> &);
  // Inserts the VLImage and finds the matches which score less than threshold
  void matchInsert(VLImage*, float threshold, std::vector<VLImage*> &);
  // match & insert wich impliments both threshold and k based matching
  void matchInsert(VLImage*, float threshold, int k, std::vector<VLImage*> &);
  // Returns the index of the first leaf
  int minLeafIndex();
  // Returns the index of the last leaf
  int maxLeafIndex();
  // Save the tree to the specified file
  void save(std::string path);
  // Load the tree from a specified file
  void load(std::string path);

//////////////////////////////////// ADITIONAL FUNCTIONS TO SAVE VT LOADED WITH IMAGES

  int leafnode;
  Eigen::SparseMatrix<float,Eigen::RowMajor> quantized_database, temp;
  // imgdesc quantized_database;
  // Save inverted files list
  void SaveInvertedFiles(std::string);
  // Save Image Descriptors
  // void SaveDescriptors(std::string);
  // Load inverted files lsit
  void LoadInvertedFiles(std::string);
  void LoadDescriptors(std::string);

  std::map<unsigned long int, int> contents;
  std::map<int, std::map<unsigned long int, int>> list;
  std::map<unsigned long int, std::string> idmap;
  std::map<unsigned long int, ImageDescriptor> descmap;
  std::vector<std::string> matchfiles;

  void MatchWithImgs(VLImage*, float threshold, int k, std::vector<unsigned long int> &matches);
  void LoadSavedImageData(const std::string& target);
 private:
    void computeWeights(std::vector<VLImage*>&);
};

}  // end of namespace: eyeMARS

#endif  // JNI_INCLUDE_VOCABTREE_H_
