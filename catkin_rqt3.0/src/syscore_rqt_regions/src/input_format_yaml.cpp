/**
  * ************************************
  * @project  catkin_rqt2.1
  * @file
  * @author   juzhango
  * @date     2020-07-26
  * @brief    format input config yaml functions
  * ************************************
  */
#include "../include/syscore_rqt_regions/input_format_yaml.h"

#include "ros/ros.h"


/**
  * @brief  Check all members
  * @param  &config: xmlrpc node
  * @param  &filter_ns: ros node namespace
  * @retval true: ok.  false: error.
  */
bool checkAvoidConfigsValidate(XmlRpc::XmlRpcValue& config, const std::string& filter_ns)
{
  /*************************** Parse the XmlRpcValue ***********************************/
  //Verify proper naming and structure
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
      ROS_ERROR("%s: The laser avoid areas specification must be a list. but is of of XmlRpcType %d", filter_ns.c_str(), config.getType());
      ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
      return false;
  }
  //Iterate over all filter in filters (may be just one)
  for(int i=0; i<config.size(); i++){
    if(config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct){
      ROS_ERROR("%s: laser avoid areas must be specified as maps, but they are XmlRpcType:%d", filter_ns.c_str(), config[i].getType());
      return false;
    }
    else if(!config[i].hasMember("name")){
      ROS_ERROR("%s: Could not add a area because no name was given", filter_ns.c_str());
      return false;
    }
    else if(!config[i].hasMember("areas")){
      ROS_ERROR("%s: Could not add a area because no areas was given", filter_ns.c_str());
      return false;
    }
    else{
      //Check for name collisions within the list itself.
      for (int j = i + 1; j < config.size(); j++){
        if(config[j].getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("%s: Filters must be specified as maps, but they are XmlRpcType:%d", filter_ns.c_str(), config[j].getType());
            return false;
        }
        if(!config[j].hasMember("name")
           ||config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString
           || config[j]["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("%s: Filters names must be strings, but they are XmlRpcTypes:%d and %d", filter_ns.c_str(), config[i].getType(), config[j].getType());
            return false;
        }
        std::string namei = config[i]["name"];
        std::string namej = config[j]["name"];
        if (namei == namej)
        {
            ROS_ERROR("%s: A self_filter with the name %s already exists", filter_ns.c_str(), namei.c_str());
            return false;
        }
      }
    }
  }
  return true;
}


/**
  * @brief  turn xmlrpc node into scene array.
  * @param  config: xmlrpc node.
  * @retval scene_vector: all scene array.
  */
std::vector<ST_SCENE> getVectorScene(XmlRpc::XmlRpcValue config)
{
  std::vector<ST_SCENE> scene_vector;
  for(int i=0; i<config.size(); i++)
  {
    ST_SCENE scene_in;
    scene_in.name = std::string(config[i]["name"]);
    ROS_INFO("load scene: %d",i+1);
    st_avoid_area_kind_ kind;
    std::string str = "polygon";
    for(int j=0; j<config[i]["areas"][kind.area_inner].size(); j++){
      std::string strPolygon = std::string(config[i]["areas"][kind.area_inner][j]["value"]);
      scene_in.areas.area_inner[str+std::to_string(j+1)] = strToPolygon(strPolygon);
      qDebug()<< "inner:" << scene_in.areas.area_inner[str+std::to_string(j+1)];
    }
    for(int j=0; j<config[i]["areas"][kind.area_mid].size(); j++){
      std::string strPolygon = std::string(config[i]["areas"][kind.area_mid][j]["value"]);
      scene_in.areas.area_mid[str+std::to_string(j+1)] = strToPolygon(strPolygon);
      qDebug()<< "mid:" << scene_in.areas.area_mid[str+std::to_string(j+1)];
    }
    for(int j=0; j<config[i]["areas"][kind.area_out].size(); j++){
      std::string strPolygon = std::string(config[i]["areas"][kind.area_out][j]["value"]);
      scene_in.areas.area_out[str+std::to_string(j+1)] = strToPolygon(strPolygon);
      qDebug()<< "out:" << scene_in.areas.area_out[str+std::to_string(j+1)];
    }
    scene_vector.push_back(scene_in);
  }
  return scene_vector;
}
/**
  * @brief  "[1, 2, 3]" >> Point(1,2); >> Polygon
  * @param  str: string
  * @retval polygon: QPolygon
  */
QPolygon strToPolygon(std::string str)
{
  QPolygon polygon;
  std::vector<std::vector<float> > vvf = parseVVF(str);
  for(int i=0; i< vvf.size(); i++)
  {
    int x = (int) (vvf[i][0]*100);
    int y = (int) (vvf[i][1]*100);
    QPoint point = QPoint(x,y);
    polygon.push_back(point);
  }
  return polygon;
}
/**
  * @brief  "[[1, 2, 3], [1, 2, 3]]" ==> vector<vector<float> >
  * @param  input: string
  * @retval result: std::vector<std::vector<float>>
  */
std::vector<std::vector<float> > parseVVF(const std::string input)
{
  std::vector<std::vector<float> > result;
  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof())
  {
    switch(input_ss.peek())
    {
    case EOF:
      break;
    case '[':
      depth++;
      if(depth>2){
        ROS_ERROR("Array depth greater than 2");
        return result;
      }
      input_ss.get();
      current_vector.clear();
      break;
    case ']':
      depth--;
      if(depth<0){
        ROS_ERROR("More close ] than open [");
        return result;
      }
      input_ss.get();
      if(depth == 1){
        result.push_back(current_vector);
      }
      break;
    case ',':
    case ' ':
    case '\t':
      input_ss.get();
      break;
    default:    // All other characters should be part of the numbers.
      if(depth !=2){
        std::stringstream err_ss;
        err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
        ROS_ERROR("%s",err_ss.str().data());
        return result;
      }
      float value;
      input_ss >> value;
      if(!!input_ss){
        current_vector.push_back(value);
      }
      break;
    }
  }
  if(depth !=0){
    ROS_ERROR("Unterminated vector string.");
  }
  return result;
}
