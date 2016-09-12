/*
 * Copyright 2016 SoftBank Robotics Europe
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "naoqi_dcm_driver/tools.hpp"

std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value, std::vector<float>& result)
{
  qi::AnyReferenceVector anyrefs = value.asListValuePtr();

  for(int i=0; i<anyrefs.size(); ++i)
  {
    try
    {
      result.push_back(anyrefs[i].content().toFloat());
    }
    catch(std::runtime_error& e)
    {
      result.push_back(-1.0);
      std::cout << e.what() << "=> set to -1" << std::endl;
    }
  }
  return result;
}

std::vector<int> fromAnyValueToIntVector(qi::AnyValue& value, std::vector<int>& result)
{
  qi::AnyReferenceVector anyrefs = value.asListValuePtr();

  for(int i=0; i<anyrefs.size();i++)
  {
    try
    {
      result.push_back(anyrefs[i].content().toInt());
    }
    catch(std::runtime_error& e)
    {
      result.push_back(-1.0);
      std::cout << e.what() << "=> set to -1" << std::endl;
    }
  }
  return result;
}
