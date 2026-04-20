#pragma once

#include <cctype>
#include <string>
#include <string_view>

namespace uav::controllers
{

inline std::string interface_to_topic(const std::string_view & name)
{
  // replace invalid characters
  std::string valid_name;
  for (const char & c : name)
  {
    if (std::isalnum(c) || c == '_' || c == '/')
    {
      valid_name.push_back(c);
    }
    else
    {
      valid_name.push_back('_');
    }
  }

  return valid_name;
}

}  // namespace uav::controllers
