#include <stdio.h>
#include "Factorial.hpp"

int main(int argc, char** argv)
{

  cpp_example::Factorial f(6);
  cpp_example::Factorial g;

  int fact_val1 = f.compute();
  int fact_val2 = g.compute();

  printf("Factorial value 1: %d\n", fact_val1);
  printf("Factorial value 2: %d\n", fact_val2);

  return 0;

}
