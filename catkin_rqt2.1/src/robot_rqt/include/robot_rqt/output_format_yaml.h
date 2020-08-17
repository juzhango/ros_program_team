#ifndef OUTPUT_FORMAT_YAML_H
#define OUTPUT_FORMAT_YAML_H
#include "yaml-cpp/yaml.h"
#include "QPolygon"
#include "vector"
#include "QString"
#include <fstream>
#include "scenetable.h"
#include "map"

struct ST_AREAS_OUT{
  std::map<std::string, std::string> area_inner;
  std::map<std::string, std::string> area_mid;
  std::map<std::string, std::string> area_out;
};
struct ST_SCENE_OUT{
  std::string name;
  bool default_scene;
  ST_AREAS_OUT areas;
};
/*
struct ST_AREAS{
  std::map<std::string, QPolygon> area_inner;
  std::map<std::string, QPolygon> area_mid;
  std::map<std::string, QPolygon> area_out;
};
struct ST_SCENE{
  std::string name;
  bool default_scene;
  ST_AREAS areas;
};
*/

std::string polygonToString(QPolygon polygon);
std::vector<ST_SCENE_OUT> scenesToScenesOut(std::vector<ST_SCENE> scenes);
YAML::Node formatYamlFile(std::vector<ST_SCENE> scenes);
void outputYamlFile(QString path, std::vector<ST_SCENE_OUT> scenes_out, QString);

std::string vector_xyzToString(std::vector <float> fPoint_xyz);
std::string vectorStrToStr(std::vector <std::string> vsPoint_xyz);
std::string polygonToString(QPolygon polygon);












#endif // OUTPUT_FORMAT_YAML_H
