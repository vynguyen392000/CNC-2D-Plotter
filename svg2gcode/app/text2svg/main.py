from ast import Return
from msilib.schema import File
import string
from numpy import str
from ttgLib.TextToGcode import ttg
str = "ab"
str = ttg(str, 24, 0, "file", 1000).toGcode("M03 S500", "M05 S0", "G0", "G1")
print(str)
