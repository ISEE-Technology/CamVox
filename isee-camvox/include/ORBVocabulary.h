
#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace Camvox
{

  typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
      ORBVocabulary;

} // namespace Camvox

#endif
