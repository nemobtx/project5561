#ifndef JNI_INCLUDE_KMEANS_H_
#define JNI_INCLUDE_KMEANS_H_

#include <time.h>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cfloat>

#include "vocab_tree/VTFeature.h"

namespace EyeMARS {

class KMeans {
 public:
    static const int MAX_ITERATIONS = 100;

    static void kMeans(std::vector<VTFeature*>& VTFeatures,
                       std::vector<VTFeature*>& means,
                       std::vector<unsigned int>& contains, unsigned int k) {
        /* initialize random seed: */
        srand(time(NULL));

	    for (std::vector<VTFeature*>::iterator it = means.begin();
	         it != means.end(); it++)
            delete *it;

        means.clear();
        contains.clear();

        // if we have less than k means pass in just clone what we have
        if (VTFeatures.size() <= k) {
            unsigned int i = 0;
            for (std::vector<VTFeature*>::iterator it = VTFeatures.begin();
                 it != VTFeatures.end(); ++it, i++) {
                means.push_back((*it)->clone());
                contains.push_back(i);
            }

            return;
        }


        // we need to select the initial means randomly
        unsigned int* indices =  new unsigned int[k];//reinterpret_cast<int*>(malloc(k*sizeof(int)));  //NOLINT
        for (int i = 0; i < static_cast<int>(k); i++) {
            unsigned int index = (rand() % VTFeatures.size());  //NOLINT
            bool found = false;
            for (unsigned int j = 0; static_cast<int>(j) < i; j++) {
                if (indices[j] == index)
                    found = true;
            }

            if (found) {
                --i;
                continue;
            }

            indices[i] = index;
            means.push_back(VTFeatures[index]->clone());
        }
        delete[] indices;
        //free(indices);

        // std::cout << "Dont initializing means" << std::endl;
        // associate each VTFeature with a mean
        contains.reserve(VTFeatures.size());
        for (unsigned int i = 0; i < VTFeatures.size(); i++) {
            unsigned int best_mean = 0, j = 0;
            float best_distance = FLT_MAX;
            for (std::vector<VTFeature*>::iterator it = means.begin();
                 it != means.end(); ++it, j++) {
                float dist = VTFeatures[i]->distance(*it);
                if (dist < best_distance) {
                    best_mean = j; 
                    best_distance = dist;
                }
            }
            contains.push_back(best_mean);
        }

        // std::cout << "All featuers associated with mean" << std::endl;
        int iterations = 0, updated = 1;
        while (updated) {
            updated = 0;

            // update groups
            int furthest_feat_idx = -1;
            float furthest_feat_distance = 0;
            for (unsigned int i = 0; i < VTFeatures.size(); i++) {
                unsigned int best_mean = 0, j = 0;
                float best_distance = FLT_MAX;
                for (std::vector<VTFeature*>::iterator it = means.begin();
                     it != means.end(); ++it, j++) {
                    float dist = VTFeatures[i]->distance(*it);
                    if (dist < best_distance) {
                        best_mean = j;
                        best_distance = dist;
                    }
                }

                // save the worst VTFeature if need later
                if (best_distance > furthest_feat_distance) {
                    furthest_feat_idx = i;
                    furthest_feat_distance = best_distance;
                }

                contains[i] = best_mean;
            }

            // std::cout << "Means updated" << std::endl;

            // compute mean
            for (unsigned int i = 0; i < means.size(); i++) {
                // find VTFeatures which are associated with this mean
                int size = 0;
                VTFeature** cluster = new VTFeature*[VTFeatures.size()];
                for (unsigned int j = 0; j < VTFeatures.size(); j++) {
                    if (contains[j] == i) {
                        cluster[size++] = VTFeatures[j];
                    }
                }

                // compute mean of cluster
                if (size > 0) {
                    // std::cout << "Computing Mean" << std::endl;

                    // std::cout << cluster[0]->type << std::endl;
                    VTFeature* new_mean = cluster[0]->ComputeMean(cluster, size);
                    // std::cout << "Mean computed" << std::endl;
                    // std::cout << "Computing distance to mean" << std::endl;
                    // std::cout << "New_mean: " << new_mean << std::endl;
                    // std::cout << "means[i]: " << means[i] << std::endl;
                    if (means[i]->distance(new_mean)) {
                        // std::cout << "New mean computed replacing it" << std::endl;
                        means[i] = new_mean;
                        updated = 1;
                    }
                    // std::cout << "same mean" << std::endl;
                } else {
                    // std::cout << "no features in cluster" << std::endl;
                    // no VTFeatures in cluster
                    if (furthest_feat_idx >= 0) {
                        means[i] = VTFeatures[furthest_feat_idx]->clone();
                        contains[furthest_feat_idx] = i;
                    }
                    updated = 1;
                }

                // std::cout << "deleting cluster" << std::endl;
                delete[] cluster;
                // std::cout << "Done deleting cluster" << std::endl;
            }

            if (iterations++ >= MAX_ITERATIONS) {
                std::cerr << "MAX ITERATIONS reached\n";
                return;
            }
        }
    }
};

}  // End of namespace: eyemars */

#endif  // JNI_INCLUDE_KMEANS_H_
