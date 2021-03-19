

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>
#include <mutex>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"  
                       
namespace Camvox
{
  class KeyFrame;
  class Frame;

  class KeyFrameDatabase
  {
  public:
    KeyFrameDatabase(const ORBVocabulary &voc);

    void add(KeyFrame *pKF);

    void erase(KeyFrame *pKF);

    void clear();

    // Loop Detection
    std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

    // Relocalization
    std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);

  protected:
    // Associated vocabulary
    const ORBVocabulary *mpVoc;

    // Inverted file
    std::vector<list<KeyFrame *>> mvInvertedFile;

    // Mutex
    std::mutex mMutex;
  };

} // namespace Camvox

#endif
