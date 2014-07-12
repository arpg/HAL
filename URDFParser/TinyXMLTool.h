#ifndef TINYXMLTOOL_H
#define TINYXMLTOOL_H

#include "/usr/local/include/tinyxml2.h"
#include <vector>
#include <string>
#include <iostream>

using namespace tinyxml2;
using namespace std;

// ---------------------------------------------------------
// get num from char
inline vector<double> GenNumFromChar(const char* numbers){
  vector<double> vNums;
  string str(numbers);

  while(str.size()!=0){
    int i=str.find_first_of(",");
    if( i>-1 ){
      string sNum = str.substr(0, i);
      float fNum = ::atof(sNum.c_str());
      vNums.push_back(fNum);

      // replace current str with substr
      str = str.substr(i+1,str.size()-i);
    }
    else if(i==-1){
      float fNum =::atof(str.c_str());
      vNums.push_back(fNum);
      break;
    }
  }
  return vNums;
}

// ------------------------------------------------------------
inline bool GetXMLdoc(string sFileName, XMLDocument& doc){
  if(doc.LoadFile(sFileName.c_str()) !=0){
    printf("Fatal Error! Cannot open %s. Please check if file is valid! \n", sFileName.c_str());
    exit(-1);
    return false;
  }
  return true;
}

// -------------------------------------------------------------
inline string GetAttribute(tinyxml2::XMLElement *pElement,
                           const char* cAttributeName){
  const char* cAttribute = pElement->Attribute(cAttributeName);
  if(cAttribute==NULL){
    cout<<"[URDFParser] Fatal error! cannot get attribute '"<<cAttributeName<<
          "' in xml file! Exit! Please check your Robot.xml file!"<<endl;
  }
  string sAttribute(cAttribute);
  return sAttribute;
}

#endif // TINYXMLTOOL_H
