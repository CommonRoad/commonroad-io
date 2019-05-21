import unittest as unittest
import os
import sys

if __name__ == "__main__":
    print(os.getcwd())
    errors = 0
    tests = 0
    for x in os.walk(os.getcwd()):
        if not '__' in x[0] and not '.' in x[0]:
            print(x[0])
            all_tests = unittest.TestLoader().discover(x[0], pattern='test_*.py')
            b = unittest.TextTestRunner().run(all_tests)
            errors += len(b.errors)
            tests += b.testsRun

    print('Executed {} tests and got {} fails'.format(tests,errors))