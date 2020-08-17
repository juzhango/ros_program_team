#ifndef INPUT_FORMAT_YAML_H
#define INPUT_FORMAT_YAML_H
#include "XmlRpc.h"
#include "QDebug"
#include "vector"
#include <iostream>

#include "../syscore_rqt_regions/scenetable.h"
#include "map"


bool checkAvoidConfigsValidate(XmlRpc::XmlRpcValue& config, const std::string& filter_ns);
std::vector<ST_SCENE> getVectorScene(XmlRpc::XmlRpcValue config);


std::vector<std::vector<float> > parseVVF(const std::string input);
QPolygon strToPolygon(std::string str);




#endif // INPUT_FORMAT_YAML_H
