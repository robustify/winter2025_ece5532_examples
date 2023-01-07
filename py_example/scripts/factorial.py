#! /usr/bin/python3

class Factorial:
    def __init__(self, n = 3):
        self.n = n

    def compute(self):
        result = 1
        for i in range(self.n):
            result = result * (self.n - i)

        return result


if __name__ == '__main__':
    f = Factorial(6)
    g = Factorial()

    fact_val1 = f.compute()
    fact_val2 = g.compute()

    print(f'Factorial value 1: {fact_val1}')
    print(f'Factorial value 2: {fact_val2}')
