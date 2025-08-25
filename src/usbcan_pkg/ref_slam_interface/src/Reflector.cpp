/*
 * @Author: LiFang6606397
 * @Date: 2024-01-08 14:55:21
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2024-07-04 09:21:50
 * @FilePath: /dev_ws/src/ref_slam_interface/src/Reflector.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */
#include "ref_slam_interface/Reflector.hpp"

namespace ref_slam_interface
{

Reflector::Reflector(){}

Reflector::~Reflector(){}

Reflector::Reflector(int id, float x, float y){
  id_ = id;
  x_ = x;
  y_ = y;
}

void 
Reflector::SetPoints(std::vector<Eigen::Vector3f> pcl){
  points_ = pcl;
}

const std::string& 
Reflector::GetName() const{
  return name_;
}

void 
Reflector::SetName(const std::string& name){
  name_ = name;
}

int 
Reflector::GetId() const{
  return id_;
}

void 
Reflector::SetId(int id){
  id_ = id;
}

float 
Reflector::GetX() const{
  return x_;
}

void 
Reflector::SetX(float x){
  x_ = x;
}

float 
Reflector::GetY() const{
  return y_;
}

void 
Reflector::SetY(float y){
  y_ = y;
}

float 
Reflector::GetRadius() const{
  return radius_;
}

void 
Reflector::SetRadius(float radius){
  radius_ = radius;
}

int 
Reflector::GetNumBeDetected() const{
  return num_be_detected_;
}

void 
Reflector::SetNumBeDetected(int num_be_detected){
  num_be_detected_ = num_be_detected;
}

const std::vector<Eigen::Vector3f>& 
Reflector::GetPoints() const{
  return points_;
}

void 
Reflector::SetPoints(const std::vector<Eigen::Vector3f>& points){
  points_ = points;
}

Eigen::Vector2d 
Reflector::GetCor() const{
  return Eigen::Vector2d(x_, y_);
}


}//namespace ref_slam_interface