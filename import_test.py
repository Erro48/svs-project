
# from modules.module_test import foo, baz
import modules.module_test as test


test.foo()

import os
os.path.splitext(os.path.basename(__file__))[0]