#include "Parameters.h"

const YAML::Node mergeNodes(const YAML::Node &defaultNode, const YAML::Node &overrideNode)
{
  if (!overrideNode.IsMap())
  {
    // If overrideNode is not a map, merge result is overrideNode, unless overrideNode is null
    return overrideNode.IsNull() ? defaultNode : overrideNode;
  }
  if (!defaultNode.IsMap())
  {
    // If defaultNode is not a map, merge result is overrideNode
    return overrideNode;
  }
  if (!defaultNode.size())
  {
    return YAML::Node(overrideNode);
  }
  // Create a new map 'newNode' with the same mappings as defaultNode, merged with overrideNode
  auto newNode = YAML::Node(YAML::NodeType::Map);
  for (auto node : defaultNode)
  {
    if (node.first.IsScalar())
    {
      const std::string &key = node.first.Scalar();
      if (overrideNode[key])
      {
        newNode[node.first] = mergeNodes(node.second, overrideNode[key]);
        continue;
      }
    }
    newNode[node.first] = node.second;
  }
  // Add the mappings from 'overrideNode' not already in 'newNode'
  for (auto node : overrideNode)
  {
    if (!node.first.IsScalar())
    {
      const std::string &key = node.first.Scalar();
      if (defaultNode[key])
      {
        newNode[node.first] = mergeNodes(defaultNode[key], node.second);
        continue;
      }
    }
    newNode[node.first] = node.second;
  }
  return YAML::Node(newNode);
}

Parameters read_parameters_from_file(string fn)
{
  return YAML::LoadFile(fn);
}

