#ifndef JNI_INCLUDE_VTFeatureLoader_H_
#define JNI_INCLUDE_VTFeatureLoader_H_

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

#include <vocab_tree/VTFeature.h>
#include <vocab_tree/BinaryVTFeature.h>
// #include <vocab_tree/FloatVTFeature.h>


/** @namespace eyemars
 * The eyemars namespace.
 */
namespace EyeMARS {

class VTFeatureLoader {
 public:
  static void save(std::ofstream& out, VTFeature* feat) {
    out << feat->type << " ";
    out << feat->index << " ";
    switch (feat->type) {
      case VTFeatureType::NONE:
        break;
      case VTFeatureType::BINARY:
        {
          BinaryVTFeature* bfeat = reinterpret_cast<BinaryVTFeature*>(feat);
          out << bfeat->getSize() << " ";
          out << std::setprecision(10) << bfeat->x() << " ";
          out << std::setprecision(10) << bfeat->y() << " ";
          BinaryVector* desc = bfeat->getDescriptor();
          for (int i = 0; i < bfeat->getSize(); i++) 
            out << (int)(*desc)(0, i) << " ";
        }
        break;
      case VTFeatureType::FLOAT:
        {
          // FloatVTFeature* ffeat = reinterpret_cast<FloatVTFeature*>(feat);
          // out << ffeat->length << " ";
          // out << std::setprecision(10) << ffeat->x << " ";
          // out << std::setprecision(10) << ffeat->y << " ";
          // for (int i = 0; i < ffeat->length; i++)
          //   out << std::setprecision(10) << ffeat->descriptor(0, i) << " ";
        }
        break;
      default:
        std::cerr << "Feature was of type: " << feat->type << " -> An Error Occured!\n";
    }
  }

  static void save(std::ofstream& out, std::vector<VTFeature*> vector) {

    out << vector.size() << " ";

    for (std::vector<VTFeature*>::iterator it = vector.begin(); it != vector.end(); ++it) {
      save(out, (*it));
      out << "\n";
    }

  }

template<typename Type>
static void WriteValueToOfstream(std::ofstream& stream_out, Type value_to_write) {
  stream_out << std::setprecision(16) << (value_to_write)  << "\n"; 
}

template<typename ReadType>
static ReadType ReadValueFromIfstream(std::ifstream& stream_in) {
 ReadType value;
 stream_in >> value;
 return value;
}
template<typename ValueType>
static void WriteStdVector(std::ofstream& stream_out, std::vector<ValueType>& _vector) {
  int size = static_cast<int>(_vector.size());
  WriteValueToOfstream<int>(stream_out,size);
  for (int i=0;i<static_cast<int>(_vector.size());i++) {
    ValueType value = _vector.at(i);
    WriteValueToOfstream<ValueType>(stream_out, value);   
  }
}


template<typename ValueType> 
static std::vector<ValueType> ReadStdVector(std::ifstream& stream_in) {
  std::vector<ValueType> vector_to_return;
  int size = ReadValueFromIfstream<int>(stream_in);
  for (int i=0;i<size;i++) {
    vector_to_return.push_back(ReadValueFromIfstream<ValueType>(stream_in));
  }
  return vector_to_return;
}


template<typename KeyType, typename ValueType>
static void WriteStdMap(std::ofstream& stream_out, std::map<KeyType,ValueType>& _map) {
  int size = static_cast<int>(_map.size());
  WriteValueToOfstream<int>(stream_out,size);
  for (int i=0;i<static_cast<int>(_map.size());i++) {
    typename  std::map<KeyType, ValueType>::iterator it = _map.begin();
    std::advance(it, i);
    WriteValueToOfstream<KeyType>(stream_out, it->first);
    WriteValueToOfstream<ValueType>(stream_out, it->second);
  }
}


template<typename KeyType, typename ValueType> 
static std::map<KeyType, ValueType> ReadStdMap(std::ifstream& stream_in) {
  std::map<KeyType, ValueType> map_to_return;
  int size = ReadValueFromIfstream<int>(stream_in);
  for (int i=0;i<size;i++) {
    map_to_return[ReadValueFromIfstream<KeyType>(stream_in)] = ReadValueFromIfstream<ValueType>(stream_in); 
  }
  return map_to_return;
}

  static void load(std::ifstream& in, std::vector<VTFeature*>& feat_vector) {
    int size;
    in >> size;
    feat_vector.resize(size);

    for (int i = 0; i < size; i++) {
      feat_vector[i] = load(in);
    }

  }

  static VTFeature* load(std::ifstream& in) {

    VTFeature* ret_feat;

    int type;
    in >> type;

    int index;
    in >> index;

    switch (type) {
      case VTFeatureType::NONE:
        ret_feat = new BinaryVTFeature();
        ret_feat->index = index;
        break;
      case VTFeatureType::BINARY:
        {
          int bytes;
          in >> bytes;
          float x, y;
          in >> std::setprecision(10) >> x >> y;
          BinaryVector desc(1, bytes);
          for (int i = 0; i < bytes; i++){
            int c_;
            in >> c_;
            desc(0, i) = c_;
          }
          ret_feat = new BinaryVTFeature(bytes, x, y, &desc);
        }
        break;
      // case VTFeatureType::FLOAT:
      //   {
      //   //   int length;
      //   //   in >> length;
      //   //   float x, y;
      //   //   in >> std::setprecision(10) >> x >> y;
      //   //   FloatVector desc(1, length);
      //   //   for(int i = 0; i < length; i++) {
      //   //     float f_;
      //   //     in >> std::setprecision(10) >>  f_;
      //   //     desc(0, i) = f_;
      //   //   }

      //   //   ret_feat = new FloatVTFeature(length, x, y, desc);
      //   }
      //   break;
      default:
        std::cerr << "Feature was of type: " << type << " -> An Error Occured!\n";
        ret_feat = new BinaryVTFeature();
    }

    return ret_feat;

  }
};

}  // end of namespace eyeMARS

#endif  // JNI_INCLUDE_VTFeatureLoader_H_
