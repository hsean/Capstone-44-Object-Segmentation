//
//  histogram_utils.h
//  Capstone44
//
//  Created by Matthew Whiteside on 5/1/16.
//
//

#include <histogram_utils.h>
using namespace boost;


template<> std::string c44::fileExt<VFH>(){ return ".vfh";};
template<> std::string c44::fileExt<CVFH>(){ return ".cvfh";};
template<> std::string c44::fileExt<OURCVFH>(){ return ".ourcvfh";};
template<> std::string c44::fileExt<ESF>(){ return ".esf";};
template<> std::string c44::fileExt<GRSD>(){ return ".grsd";};

template<> std::string c44::fieldName<VFH>(){ return "vfh";};
template<> std::string c44::fieldName<CVFH>(){ return "vfh";};
template<> std::string c44::fieldName<OURCVFH>(){ return "vfh";};
template<> std::string c44::fieldName<ESF>(){ return "esf";};
template<> std::string c44::fieldName<GRSD>(){ return "grsd";};


