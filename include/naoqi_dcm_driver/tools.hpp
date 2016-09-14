#ifndef TOOLS_HPP
#define TOOLS_HPP

// NAOqi Headers
#include <qi/anyvalue.hpp>

qi::AnyValue fromStringVectorToAnyValue(const std::vector<std::string> &vector);

qi::AnyValue fromDoubleVectorToAnyValue(const std::vector<double> &vector);

std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value);

std::vector<int> fromAnyValueToIntVector(qi::AnyValue& value);

std::string print(const std::vector <std::string> &vector);

#endif // TOOLS_HPP
