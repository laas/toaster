#include <string>
#include <vector>

using namespace std;

#ifndef XMLUTILITY_H
#define XMLUTILITY_H

vector<string> loadPropertiesFromXml(string objectID);

string loadValueFromXmlAsString(string objectID, string factName, string objectValueName, string objectValue);

#endif
