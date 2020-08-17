/**
  * ************************************
  * @roject  catkin_rqt2.1
  * @file
  * @author  juzhango
  * @date    2020-07-26
  * @brief
  * ************************************
  */
#include "../include/robot_rqt/output_format_yaml.h"
#include "../include/robot_rqt/main_window.hpp"
#include "iostream"
#include "../include/robot_rqt/common.h"

std::string vector_xyzToString(std::vector <float> fPoint_xyz)
{
  std::string str;
  std::ostringstream oss_x;
  std::ostringstream oss_y;
  std::ostringstream oss_z;

  oss_x << fPoint_xyz.at(0);
  oss_y << fPoint_xyz.at(1);
  oss_z << fPoint_xyz.at(2);

  str.append(1,'[');
  str = str + oss_x.str() + ", " + oss_y.str() + ", " + oss_z.str();
  str.append(1,']');

  return str;
}
std::string vectorStrToStr(std::vector <std::string> vsPoint_xyz)
{
  std::string str;
  str.append(1,'[');
  str = str + vsPoint_xyz.at(0);
  for(int i = 1; i < vsPoint_xyz.size(); i++)
  {
    str = str + ", " + vsPoint_xyz.at(i);
  }
  str.append(1,']');
  return str;
}
// QPolygon => "[[1, 2, 0.1], [2, 1, 0.1]]"
std::string polygonToString(QPolygon polygon)
{
  std::string sPoint_xyz_str; //string: "[[1, 2, 0.1], [2, 1, 0.1]]"
  std::vector <std::string> vsPoint_xyz;  //array: {"[0.03, 1.24, 0.1]", "[0.03, 1.24, 0.1]"}
  if(polygon.size()<1)
  {
    polygon.push_back(QPoint(0,0));
  }
  for(int i = 0; i < polygon.size(); i++)     // one point
  {
    std::vector <float> fPoint_xyz; //array: {x, y, z}
    std::string sPoint_xyz;     // "[0.03, 1.24, 0.1]"
    float x = float(polygon.at(i).x())  / 100;
    float y = float(polygon.at(i).y())  / 100;
    float z = 0.1;

    fPoint_xyz.push_back(x);
    fPoint_xyz.push_back(y);
    fPoint_xyz.push_back(z);
    sPoint_xyz = vector_xyzToString(fPoint_xyz);
    vsPoint_xyz.push_back(sPoint_xyz);
  }
  sPoint_xyz_str = vectorStrToStr(vsPoint_xyz);
  return sPoint_xyz_str;
}
/**
  * @brief  QPolygon => std::string
  * @param  scenes: QPolygon type
  * @retval std::vector<ST_SCENE_OUT>
  */
std::vector<ST_SCENE_OUT> scenesToScenesOut(std::vector<ST_SCENE> scenes)
{
  qDebug()<<"enter: scenesToScenesOut()";
  std::vector<ST_SCENE_OUT> scenes_out_vector;

  for(int i = 0; i < scenes.size(); i++){
    ST_SCENE_OUT scene_output;
    scene_output.name = scenes.at(i).name;
    scene_output.default_scene = scenes.at(i).default_scene;
    std::string str = "polygon";

    for(int j = 0; j < scenes.at(i).areas.area_inner[str+std::to_string(j+1)].size(); j++){
      scene_output.areas.area_inner[str+std::to_string(j+1)] = polygonToString(scenes.at(i).areas.area_inner[str+std::to_string(j+1)]);
      qDebug() << "scene inner : " << QString::fromStdString(scene_output.areas.area_inner[str+std::to_string(j+1)]);
    }
    for(int j = 0; j < scenes.at(i).areas.area_mid[str+std::to_string(j+1)].size(); j++){
      scene_output.areas.area_mid[str+std::to_string(j+1)] = polygonToString(scenes.at(i).areas.area_mid[str+std::to_string(j+1)]);
      qDebug() << "scene mid: " << QString::fromStdString(scene_output.areas.area_mid[str+std::to_string(j+1)]);
    }
    for(int j = 0; j < scenes.at(i).areas.area_out[str+std::to_string(j+1)].size(); j++){
      scene_output.areas.area_out[str+std::to_string(j+1)] = polygonToString(scenes.at(i).areas.area_out[str+std::to_string(j+1)]);
      qDebug() << "scene out: " << QString::fromStdString(scene_output.areas.area_out[str+std::to_string(j+1)]);
    }
    scenes_out_vector.push_back(scene_output);
  }
  return scenes_out_vector;
}
YAML::Node generateYamlNode(std::vector<ST_SCENE_OUT> scenes_output)
{
  YAML::Node node;
  for(int i = 0; i < scenes_output.size(); i++){
    node[i]["name"] = scenes_output.at(i).name;
    node[i]["default_scene"] = scenes_output.at(i).default_scene;
    std::string str = "polygon";
    for(int j = 0; j < scenes_output.at(i).areas.area_inner[str+std::to_string(j+1)].size(); j++){
      node[i]["areas"]["area_inner"][j]["name"] = str+std::to_string(j+1);
      node[i]["areas"]["area_inner"][j]["value"] = scenes_output.at(i).areas.area_inner[str+std::to_string(j+1)];;
    }
    for(int j = 0; j < scenes_output.at(i).areas.area_mid[str+std::to_string(j+1)].size(); j++){
      node[i]["areas"]["area_mid"][j]["name"] = str+std::to_string(j+1);
      node[i]["areas"]["area_mid"][j]["value"] = scenes_output.at(i).areas.area_mid[str+std::to_string(j+1)];;
    }
    for(int j = 0; j < scenes_output.at(i).areas.area_out[str+std::to_string(j+1)].size(); j++){
      node[i]["areas"]["area_out"][j]["name"] = str+std::to_string(j+1);
      node[i]["areas"]["area_out"][j]["value"] = scenes_output.at(i).areas.area_out[str+std::to_string(j+1)];;
    }
  }
  return node;
}
/**
  * @brief  output yaml file
  * @param  path: Path of opened file + file name
  * @param  scenes_out:
  * @retval None
  */
void outputYamlFile(QString path, std::vector<ST_SCENE_OUT> scenes_out, QString paramName)
{
  std::string str = paramName.toStdString() + ":\n";
  std::ofstream fout(path.toStdString());
  fout << str;
  fout << generateYamlNode(scenes_out);
  fout.close();
}
