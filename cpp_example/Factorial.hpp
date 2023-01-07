#pragma once

namespace cpp_example {

  class Factorial {
    public:
      Factorial(int n = 3);
      int compute();

    private:
      int n_;

  };

}
