// Copyright (c) bminortx

#ifndef URDFPARSER_TINYXMLTOOL_H_
#define URDFPARSER_TINYXMLTOOL_H_

#include <vector>
#include <string>

#include <tinyxml2.h>
#include <miniglog/logging.h>

// get num from char
inline std::vector<double> GenNumFromChar(const char* numbers) {
  std::vector<double> vNums;
  std::string str(numbers);

  while (str.size()!= 0) {
    int i = str.find_first_of(", ");
    if (i > -1) {
      std::string sNum = str.substr(0, i);
      float fNum = ::atof(sNum.c_str());
      vNums.push_back(fNum);

      //  replace current str with substr
      str = str.substr(i+1, str.size()-i);
    } else if (i == -1) {
      float fNum = ::atof(str.c_str());
      vNums.push_back(fNum);
      break;
    }
  }
  return vNums;
}

inline bool GetXMLdoc(std::string sFileName, tinyxml2::XMLDocument& doc) {
  if (doc.LoadFile(sFileName.c_str()) != 0) {
    LOG(FATAL) << "Fatal Error! Cannot open %s. Please check if file is valid: "
              << sFileName.c_str();
    exit(-1);
    return false;
  }
  return true;
}

inline std::string GetAttribute(tinyxml2::XMLElement *pElement,
                           const char* cAttributeName) {
  const char* cAttribute = pElement->Attribute(cAttributeName);
  if (cAttribute == NULL) {
    LOG(FATAL) << "[URDFParser] Fatal error! cannot get attribute '"
               << cAttributeName
               << "' in xml file! Exit! Please check your Robot.xml file!";
  }
  std::string sAttribute(cAttribute);
  return sAttribute;
}

#endif  // URDFPARSER_TINYXMLTOOL_H_
