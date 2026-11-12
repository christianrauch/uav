#include <cstdio>
#include <uav_autotune/filter/AlphaFilter.hpp>
#include <uav_autotune/pid_design/pid_design.hpp>

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  printf("hello world uav_autotune_node package\n");

  AlphaFilter<float> _signal_filter;
}
